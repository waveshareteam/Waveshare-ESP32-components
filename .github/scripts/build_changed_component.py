#!/usr/bin/env python3
import os
import shutil
import subprocess
import sys
from pathlib import Path


REPO = Path.cwd()
DEFAULT_TARGET = "esp32s3"
LOCAL_COMPONENT_ROOTS = ("display/lcd", "display/oled", "display/touch", "sensor", "lora")


def idf_command():
    found = shutil.which("idf.py")
    if found:
        return [found]
    idf_path = os.environ.get("IDF_PATH")
    if idf_path:
        fallback = Path(idf_path) / "tools" / "idf.py"
        if fallback.exists():
            return [sys.executable, str(fallback)]
    print("idf.py was not found in PATH or $IDF_PATH/tools", file=sys.stderr)
    sys.exit(1)


def strip_scalar(value):
    value = value.strip()
    if not value:
        return ""
    if (value[0] == value[-1]) and value[0] in ("'", '"'):
        return value[1:-1]
    return value


def component_targets(component_dir):
    manifest = component_dir / "idf_component.yml"
    lines = manifest.read_text(encoding="utf-8").splitlines()
    targets = []
    for line_number, line in enumerate(lines):
        if line.strip() != "targets:" or line.startswith((" ", "\t")):
            continue
        for child in lines[line_number + 1:]:
            child_stripped = child.strip()
            if not child_stripped:
                continue
            if not child.startswith((" ", "\t")) and not child_stripped.startswith("-"):
                break
            if child_stripped.startswith("-"):
                targets.append(strip_scalar(child_stripped[1:]))
        break
    return targets or [DEFAULT_TARGET]


def component_dependencies(component_dir):
    manifest = component_dir / "idf_component.yml"
    lines = manifest.read_text(encoding="utf-8").splitlines()
    dependencies = []
    for line_number, line in enumerate(lines):
        if line.strip() != "dependencies:" or line.startswith((" ", "\t")):
            continue
        for child in lines[line_number + 1:]:
            child_stripped = child.strip()
            if not child_stripped or child_stripped.startswith("#"):
                continue
            if not child.startswith((" ", "\t")):
                break
            if child.startswith("  ") and not child.startswith("    ") and ":" in child_stripped:
                dependencies.append(child_stripped.split(":", 1)[0])
        break
    return dependencies


def local_component_index():
    index = {}
    for root in LOCAL_COMPONENT_ROOTS:
        root_path = REPO / root
        if not root_path.exists():
            continue
        for manifest in root_path.glob("*/idf_component.yml"):
            component_dir = manifest.parent
            component_name = component_dir.name
            for key in (component_name, f"waveshare/{component_name}", f"espressif/{component_name}"):
                index.setdefault(key, component_dir)
    return index


def local_dependency_overrides(component_dir):
    index = local_component_index()
    overrides = {}
    for dependency in component_dependencies(component_dir):
        if dependency in index:
            overrides[dependency] = index[dependency]
    return overrides


def candidate_projects(component_dir):
    projects = []
    for name in ("test_app", "test_apps"):
        candidate = component_dir / name
        if (candidate / "CMakeLists.txt").exists():
            projects.append(candidate)
    return projects


def write_generated_project(component_dir, component_slug):
    component_name = component_dir.name
    project_dir = REPO / ".component_self_check" / component_slug
    if project_dir.exists():
        shutil.rmtree(project_dir)
    (project_dir / "main").mkdir(parents=True)
    extra_component_dirs = [component_dir.resolve()]
    local_overrides = local_dependency_overrides(component_dir)
    for dependency_dir in local_overrides.values():
        if dependency_dir.resolve() not in extra_component_dirs:
            extra_component_dirs.append(dependency_dir.resolve())
    extra_component_dir_lines = "\n".join(f'    "{path.as_posix()}"' for path in extra_component_dirs)

    (project_dir / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.16)\n"
        "set(EXTRA_COMPONENT_DIRS\n"
        f"{extra_component_dir_lines}\n"
        ")\n"
        "include($ENV{IDF_PATH}/tools/cmake/project.cmake)\n"
        "project(component_self_check)\n",
        encoding="utf-8",
    )
    (project_dir / "main" / "CMakeLists.txt").write_text(
        f'idf_component_register(SRCS "main.c" REQUIRES {component_name})\n',
        encoding="utf-8",
    )
    (project_dir / "main" / "main.c").write_text(
        "void app_main(void)\n"
        "{\n"
        "}\n",
        encoding="utf-8",
    )
    if local_overrides:
        manifest_lines = ["dependencies:"]
        for dependency, dependency_dir in sorted(local_overrides.items()):
            relative_path = os.path.relpath(dependency_dir, project_dir / "main").replace(os.sep, "/")
            manifest_lines.extend((f"  {dependency}:", f'    path: "{relative_path}"'))
        (project_dir / "main" / "idf_component.yml").write_text(
            "\n".join(manifest_lines) + "\n",
            encoding="utf-8",
        )
    return project_dir


def run_build(project_dir, target, build_root):
    build_dir = build_root / project_dir.name / target
    command = [
        *idf_command(),
        "-C",
        str(project_dir),
        "-B",
        str(build_dir),
        "set-target",
        target,
        "build",
    ]
    print("Running:", " ".join(command), flush=True)
    subprocess.run(command, cwd=REPO, check=True)


def main():
    component = os.environ.get("COMPONENT_DIR")
    if not component:
        print("COMPONENT_DIR is required", file=sys.stderr)
        sys.exit(1)

    component_dir = REPO / component
    if not (component_dir / "idf_component.yml").exists():
        print(f"{component}: missing idf_component.yml", file=sys.stderr)
        sys.exit(1)

    projects = candidate_projects(component_dir)
    if not projects:
        projects = [write_generated_project(component_dir, component.replace("/", "__"))]

    build_root = REPO / ".component_self_check_build"
    for target in component_targets(component_dir):
        for project in projects:
            run_build(project, target, build_root)


if __name__ == "__main__":
    main()
