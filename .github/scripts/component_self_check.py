#!/usr/bin/env python3
import json
import os
import re
import subprocess
import sys
from pathlib import Path


REPO = Path.cwd()
SEMVER_RE = re.compile(r"^(\d+)\.(\d+)\.(\d+)(?:[-+].*)?$")
COMPONENT_ROOTS = ("bsp", "sensor", "display/lcd", "display/oled", "display/touch", "lora")


def run_git(*args, check=True):
    result = subprocess.run(["git", *args], cwd=REPO, text=True, capture_output=True)
    if check and result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip())
    return result.stdout


def strip_scalar(value):
    value = value.strip()
    if not value:
        return ""
    if (value[0] == value[-1]) and value[0] in ("'", '"'):
        return value[1:-1]
    return value


def parse_manifest_text(text, source):
    data = {}
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or line.startswith((" ", "\t")):
            i += 1
            continue

        if stripped.startswith("version:"):
            data["version"] = strip_scalar(stripped.split(":", 1)[1])
        elif stripped == "targets:":
            targets = []
            i += 1
            while i < len(lines):
                child = lines[i]
                child_stripped = child.strip()
                if not child_stripped:
                    i += 1
                    continue
                if not child.startswith((" ", "\t")) and not child_stripped.startswith("-"):
                    i -= 1
                    break
                if child_stripped.startswith("-"):
                    targets.append(strip_scalar(child_stripped[1:]))
                i += 1
            data["targets"] = targets
        elif stripped == "dependencies:":
            dependencies = {}
            i += 1
            while i < len(lines):
                dep_line = lines[i]
                dep_stripped = dep_line.strip()
                if not dep_stripped:
                    i += 1
                    continue
                if not dep_line.startswith((" ", "\t")):
                    i -= 1
                    break
                if dep_line.startswith("  ") and not dep_line.startswith("    ") and ":" in dep_stripped:
                    dep_name, dep_value = dep_stripped.split(":", 1)
                    dep_value = dep_value.strip()
                    if dep_value:
                        dependencies[dep_name] = strip_scalar(dep_value)
                    else:
                        nested = {}
                        i += 1
                        while i < len(lines):
                            nested_line = lines[i]
                            nested_stripped = nested_line.strip()
                            if not nested_stripped:
                                i += 1
                                continue
                            if not nested_line.startswith("    "):
                                i -= 1
                                break
                            if ":" in nested_stripped:
                                nested_key, nested_value = nested_stripped.split(":", 1)
                                nested[nested_key] = strip_scalar(nested_value)
                            i += 1
                        dependencies[dep_name] = nested
                i += 1
            data["dependencies"] = dependencies
        i += 1

    if "version" not in data and "dependencies" not in data:
        raise RuntimeError(f"{source}: could not parse expected manifest fields")
    return data


def load_manifest_file(path):
    return parse_manifest_text(path.read_text(encoding="utf-8"), str(path))


def parse_upload_dirs():
    workflow_lines = (REPO / ".github" / "workflows" / "upload_component.yml").read_text(encoding="utf-8").splitlines()
    directories = []
    for line_number, line in enumerate(workflow_lines):
        if line.strip() != "directories: >":
            continue
        indent = len(line) - len(line.lstrip())
        for child in workflow_lines[line_number + 1:]:
            child_stripped = child.strip()
            child_indent = len(child) - len(child.lstrip())
            if child_stripped and child_indent <= indent:
                break
            if child_stripped:
                directories.append(child_stripped)
        break
    if not directories:
        raise RuntimeError("upload_component.yml does not define component directories")
    return {item.strip() for item in "\n".join(directories).split(";") if item.strip()}


def is_test_manifest(path):
    parts = set(path.parts)
    return "test_app" in parts or "test_apps" in parts


def discover_component_dirs():
    component_dirs = set()
    for root in COMPONENT_ROOTS:
        root_path = REPO / root
        if not root_path.exists():
            continue
        for manifest in root_path.rglob("idf_component.yml"):
            if is_test_manifest(manifest.relative_to(REPO)):
                continue
            component_dirs.add(manifest.parent.relative_to(REPO).as_posix())
    return component_dirs


def resolve_base_ref():
    event_name = os.environ.get("EVENT_NAME", "")
    if event_name == "pull_request":
        base = os.environ.get("PR_BASE_SHA")
        if base:
            return base
    if event_name == "push":
        before = os.environ.get("PUSH_BEFORE_SHA")
        if before and not before.startswith("0" * 40):
            return before
    return run_git("rev-parse", "HEAD~1").strip()


def changed_files(base_ref):
    output = run_git("diff", "--name-only", "--diff-filter=ACMRT", f"{base_ref}..HEAD")
    return [line.strip() for line in output.splitlines() if line.strip()]


def changed_components(files, component_dirs):
    changed = set()
    for file_name in files:
        for component_dir in component_dirs:
            if file_name == component_dir or file_name.startswith(component_dir + "/"):
                changed.add(component_dir)
                break
    return sorted(changed)


def build_components(changed, component_dirs, upload_dirs):
    scope = os.environ.get("BUILD_COMPONENT_SCOPE", "changed").strip().lower()
    if scope in ("all", "full"):
        return sorted(component_dirs)
    if scope in ("changed", ""):
        return list(changed)
    raise RuntimeError(f"BUILD_COMPONENT_SCOPE must be 'changed' or 'all', got {scope!r}")


def manifest_at(ref, component_dir):
    manifest_path = f"{component_dir}/idf_component.yml"
    result = subprocess.run(
        ["git", "show", f"{ref}:{manifest_path}"],
        cwd=REPO,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        return None
    return parse_manifest_text(result.stdout, f"{ref}:{manifest_path}")


def semver_tuple(value, source):
    text = str(value)
    match = SEMVER_RE.match(text)
    if not match:
        raise RuntimeError(f"{source}: version must be SemVer x.y.z, got {text!r}")
    return tuple(int(part) for part in match.groups())


def check_version_bumps(base_ref, components):
    errors = []
    for component_dir in components:
        manifest_path = REPO / component_dir / "idf_component.yml"
        current = load_manifest_file(manifest_path)
        current_version = current.get("version")
        if current_version is None:
            errors.append(f"{component_dir}: missing top-level version")
            continue

        previous = manifest_at(base_ref, component_dir)
        if previous is None:
            print(f"{component_dir}: new component, version {current_version}")
            continue

        previous_version = previous.get("version")
        if previous_version is None:
            print(f"{component_dir}: previous manifest had no version, current {current_version}")
            continue

        try:
            current_tuple = semver_tuple(current_version, str(manifest_path))
            previous_tuple = semver_tuple(previous_version, f"{base_ref}:{component_dir}/idf_component.yml")
        except RuntimeError as exc:
            errors.append(str(exc))
            continue

        if current_tuple <= previous_tuple:
            errors.append(
                f"{component_dir}: version must increase when component files change "
                f"({previous_version} -> {current_version})"
            )
        else:
            print(f"{component_dir}: version bump {previous_version} -> {current_version}")
    return errors


def check_bsp_codec_dependency():
    errors = []
    for manifest_path in sorted((REPO / "bsp").glob("*/idf_component.yml")):
        manifest = load_manifest_file(manifest_path)
        dependencies = manifest.get("dependencies") or {}
        codec_dep = dependencies.get("esp_codec_dev")
        if codec_dep is None:
            continue
        version = codec_dep.get("version") if isinstance(codec_dep, dict) else codec_dep
        if str(version) != "~1.5":
            errors.append(f"{manifest_path.relative_to(REPO)}: esp_codec_dev version must be \"~1.5\"")
    return errors


def write_outputs(components):
    output_path = os.environ.get("GITHUB_OUTPUT")
    if not output_path:
        return
    with open(output_path, "a", encoding="utf-8") as output:
        output.write(f"components={json.dumps(components)}\n")
        output.write(f"count={len(components)}\n")


def main():
    upload_dirs = parse_upload_dirs()
    component_dirs = discover_component_dirs()
    base_ref = resolve_base_ref()
    files = changed_files(base_ref)
    components = changed_components(files, component_dirs)
    output_components = build_components(components, component_dirs, upload_dirs)

    unlisted = sorted(set(components) - upload_dirs)
    output_unlisted = sorted(set(output_components) - upload_dirs)
    errors = []
    if unlisted:
        errors.extend(f"{item}: component is not listed in upload_component.yml" for item in unlisted)
    if output_unlisted:
        errors.extend(f"{item}: build component is not listed in upload_component.yml" for item in output_unlisted)

    errors.extend(check_version_bumps(base_ref, components))
    errors.extend(check_bsp_codec_dependency())
    write_outputs(output_components)

    print(f"Base ref: {base_ref}")
    print(f"Changed files: {len(files)}")
    print(f"Changed components: {len(components)}")
    for component in components:
        print(f"  - {component}")
    print(f"Build components: {len(output_components)}")
    for component in output_components:
        print(f"  - {component}")

    if errors:
        print("\nManifest check failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
