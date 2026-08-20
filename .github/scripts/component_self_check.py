#!/usr/bin/env python3
import json
import os
import re
import subprocess
import sys
from pathlib import Path, PurePosixPath


REPO = Path.cwd()
SEMVER_RE = re.compile(r"^(\d+)\.(\d+)\.(\d+)(?:[-+].*)?$")
COMPONENT_ROOTS = ("bsp", "sensor", "display/lcd", "display/oled", "display/touch", "lora")
LCD5_BSP_DIR = "bsp/esp32_p4_wifi6_touch_lcd_5"
HX8394_DIR = "display/lcd/esp_lcd_hx8394"
LCD5_REQUIRED_FILES = {
    "CMakeLists.txt",
    "Kconfig",
    "LICENSE",
    "README.md",
    "esp32_p4_wifi6_touch_lcd_5.c",
    "idf_component.yml",
    "include/bsp/config.h",
    "include/bsp/display.h",
    "include/bsp/esp-bsp.h",
    "include/bsp/esp32_p4_wifi6_touch_lcd_5.h",
    "include/bsp/touch.h",
    "priv_include/bsp_err_check.h",
}


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
        elif stripped.startswith("repository:"):
            data["repository"] = strip_scalar(stripped.split(":", 1)[1])
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
                if not dep_stripped or dep_stripped.startswith("#"):
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
                            if not nested_stripped or nested_stripped.startswith("#"):
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


def check_lcd5_hx8394_contract(upload_dirs):
    """Keep the LCD-5 BSP and HX8394 driver integration explicit and reviewable."""
    errors = []
    hx_dir = REPO / HX8394_DIR
    bsp_dir = REPO / LCD5_BSP_DIR
    hx_manifest = load_manifest_file(hx_dir / "idf_component.yml")
    bsp_manifest = load_manifest_file(bsp_dir / "idf_component.yml")
    hx_kconfig = (hx_dir / "Kconfig").read_text(encoding="utf-8")
    bsp_kconfig = (bsp_dir / "Kconfig").read_text(encoding="utf-8")
    hx_source = (hx_dir / "esp_lcd_hx8394.c").read_text(encoding="utf-8")
    hx_readme = (hx_dir / "README.md").read_text(encoding="utf-8")
    bsp_readme = (bsp_dir / "README.md").read_text(encoding="utf-8")
    root_readme = (REPO / "README.md").read_text(encoding="utf-8")

    try:
        if semver_tuple(hx_manifest.get("version", ""), f"{HX8394_DIR}/idf_component.yml") < (2, 1, 0):
            errors.append(f"{HX8394_DIR}/idf_component.yml: expected version >= 2.1.0")
    except RuntimeError as exc:
        errors.append(str(exc))
    if not re.search(r"config ESP_LCD_HX8394_SKIP_I2C_INIT\s+bool.*?default n", hx_kconfig, re.DOTALL):
        errors.append(f"{HX8394_DIR}/Kconfig: missing ESP_LCD_HX8394_SKIP_I2C_INIT default n")

    guard_start = hx_source.find("#if !CONFIG_ESP_LCD_HX8394_SKIP_I2C_INIT")
    guard_end = hx_source.find("#endif // !CONFIG_ESP_LCD_HX8394_SKIP_I2C_INIT", guard_start)
    if guard_start < 0 or guard_end < 0:
        errors.append(f"{HX8394_DIR}/esp_lcd_hx8394.c: missing complete I2C opt-out guard")
    else:
        for token in (
            "i2c_config_t conf",
            "i2c_bus_create",
            "i2c_bus_device_create",
            "i2c_bus_write_bytes",
            "i2c_bus_device_delete",
            "vTaskDelay(pdMS_TO_TICKS(1000))",
        ):
            position = hx_source.find(token)
            if not guard_start < position < guard_end:
                errors.append(f"{HX8394_DIR}/esp_lcd_hx8394.c: {token} is outside the I2C opt-out guard")
        if "#include \"sdkconfig.h\"" not in hx_source or "#ifndef CONFIG_ESP_LCD_HX8394_SKIP_I2C_INIT" not in hx_source:
            errors.append(f"{HX8394_DIR}/esp_lcd_hx8394.c: missing generated-config fallback")

    dependencies = bsp_manifest.get("dependencies") or {}
    try:
        if semver_tuple(bsp_manifest.get("version", ""), f"{LCD5_BSP_DIR}/idf_component.yml") < (1, 0, 1):
            errors.append(f"{LCD5_BSP_DIR}/idf_component.yml: expected version >= 1.0.1")
    except RuntimeError as exc:
        errors.append(str(exc))
    if bsp_manifest.get("targets") != ["esp32p4"]:
        errors.append(f"{LCD5_BSP_DIR}/idf_component.yml: expected esp32p4 target")
    if dependencies.get("idf") != ">=5.4":
        errors.append(f"{LCD5_BSP_DIR}/idf_component.yml: expected idf >=5.4")
    codec_dep = dependencies.get("esp_codec_dev")
    codec_version = codec_dep.get("version") if isinstance(codec_dep, dict) else codec_dep
    if codec_version != "~1.5":
        errors.append(f"{LCD5_BSP_DIR}/idf_component.yml: expected esp_codec_dev ~1.5")
    hx_dependency = dependencies.get("waveshare/esp_lcd_hx8394")
    # Accept either the immutable git pin (PR/HIL validation phase) or the
    # published registry release. The registry rejects components whose
    # dependencies are git-pinned, so released BSP versions must resolve
    # esp_lcd_hx8394 from the registry.
    if hx_dependency != {
        "git": "https://github.com/waveshareteam/Waveshare-ESP32-components.git",
        "path": "display/lcd/esp_lcd_hx8394",
        "version": "fc6e6d2d63aa314cdcec2e8912614aacff2fbd6d",
    } and hx_dependency != "^2.1.0":
        errors.append(
            f"{LCD5_BSP_DIR}/idf_component.yml: expected waveshare/esp_lcd_hx8394"
            " ^2.1.0 registry dependency (or the immutable HX8394 PR/HIL git pin)"
        )
    if bsp_manifest.get("repository") != "https://github.com/waveshareteam/Waveshare-ESP32-components.git":
        errors.append(f"{LCD5_BSP_DIR}/idf_component.yml: expected canonical repository URL")
    bridge = re.compile(
        r"config BSP_LCD_HX8394_SKIP_I2C_INIT\s+bool\s+default y\s+select ESP_LCD_HX8394_SKIP_I2C_INIT",
        re.DOTALL,
    )
    if not bridge.search(bsp_kconfig):
        errors.append(f"{LCD5_BSP_DIR}/Kconfig: missing default-y HX8394 opt-out bridge")

    actual_files = {
        path.relative_to(bsp_dir).as_posix()
        for path in bsp_dir.rglob("*")
        if path.is_file()
    }
    missing_files = sorted(LCD5_REQUIRED_FILES - actual_files)
    if missing_files:
        errors.append(f"{LCD5_BSP_DIR}: missing required source BSP files: {', '.join(missing_files)}")
    if LCD5_BSP_DIR not in upload_dirs:
        errors.append(f"upload_component.yml: missing {LCD5_BSP_DIR}")
    if "ESP32-P4-WIFI6-Touch-LCD-5" not in root_readme:
        errors.append("README.md: missing LCD-5 BSP table entry")
    if "ESP_LCD_HX8394_SKIP_I2C_INIT" not in hx_readme or "default" not in hx_readme:
        errors.append(f"{HX8394_DIR}/README.md: missing default/opt-out documentation")
    if "BSP_LCD_HX8394_SKIP_I2C_INIT" not in bsp_readme:
        errors.append(f"{LCD5_BSP_DIR}/README.md: missing BSP opt-out rationale")
    return errors


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
    # Three-dot range: diff from the merge base so only the PR's own changes
    # are considered. A two-dot range also reports files the base branch
    # gained since the PR last merged it, which wrongly looks like the PR
    # touched CI/global files and forces a full build.
    output = run_git("diff", "--name-only", "--diff-filter=ACMRT", f"{base_ref}...HEAD")
    return [line.strip() for line in output.splitlines() if line.strip()]


BUILD_AFFECTING_GLOBALS = (
    ".github/scripts/",
    ".github/workflows/component_self_check.yml",
    ".gitmodules",
)


def is_build_affecting_global(path_text):
    """Global files that can change how components are built or checked.

    Changes to the upload list (upload_component.yml) only affect publishing,
    not builds, so they do not force a full build.
    """
    return any(marker in path_text for marker in BUILD_AFFECTING_GLOBALS)


def dependency_closure(changed, component_dirs):
    """Expand the changed set with in-repo components that depend on it.

    Matches both registry-style dependencies (waveshare/<name>) and git
    dependencies pinned by repository path. Iterates so a change to a shared
    driver also rebuilds the BSPs that (transitively) use it.
    """
    result = set(changed)
    pending = list(changed)
    while pending:
        current = pending.pop(0)
        base_name = PurePosixPath(current).name
        for directory in component_dirs:
            if directory in result:
                continue
            manifest_path = REPO / directory / "idf_component.yml"
            try:
                manifest = load_manifest_file(manifest_path)
            except RuntimeError:
                continue
            for dep_name, dep_value in (manifest.get("dependencies") or {}).items():
                depends = False
                if dep_name == f"waveshare/{base_name}":
                    depends = True
                elif isinstance(dep_value, dict) and str(dep_value.get("path", "")).strip() == current:
                    depends = True
                if depends:
                    result.add(directory)
                    pending.append(directory)
                    break
    return sorted(result)


def split_changed_files(files, component_dirs):
    changed = set()
    global_files = []
    sorted_component_dirs = sorted(component_dirs)
    for file_name in files:
        for component_dir in sorted_component_dirs:
            if file_name == component_dir or file_name.startswith(component_dir + "/"):
                changed.add(component_dir)
                break
        else:
            global_files.append(file_name)
    return sorted(changed), global_files


def build_components(changed, component_dirs, global_files):
    scope = os.environ.get("BUILD_COMPONENT_SCOPE", "auto").strip().lower()
    if scope in ("all", "full"):
        return sorted(component_dirs)
    closed = dependency_closure(changed, component_dirs)
    if scope in ("changed", ""):
        return closed
    if scope == "auto":
        # Global changes that do not affect builds (docs, licenses, the
        # upload list) keep the scoped build. Changes to CI scripts or the
        # build workflow still validate every component once, as they may
        # alter how components are built or checked.
        if not global_files or not any(is_build_affecting_global(f) for f in global_files):
            return closed
        return sorted(component_dirs)
    raise RuntimeError(f"BUILD_COMPONENT_SCOPE must be 'auto', 'changed', or 'all', got {scope!r}")


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


def check_bsp_codec_dependency(components):
    errors = []
    manifests = []
    for component_dir in components:
        if component_dir.startswith("bsp/"):
            manifests.append(REPO / component_dir / "idf_component.yml")
    for manifest_path in sorted(manifests):
        manifest = load_manifest_file(manifest_path)
        dependencies = manifest.get("dependencies") or {}
        codec_dep = dependencies.get("esp_codec_dev")
        if codec_dep is None:
            continue
        version = codec_dep.get("version") if isinstance(codec_dep, dict) else codec_dep
        if str(version) != "~1.5":
            errors.append(f"{manifest_path.relative_to(REPO)}: esp_codec_dev version must be \"~1.5\"")
    return errors


def check_p4_lcd_x_panel_dependencies(components):
    component_dir = "bsp/esp32_p4_wifi6_touch_lcd_x"
    if component_dir not in components:
        return []

    manifest_path = REPO / component_dir / "idf_component.yml"
    dependencies = load_manifest_file(manifest_path).get("dependencies") or {}
    expected = {
        "esp_lcd_jd9365": "^2.0.2",
        "waveshare/esp_lcd_ili9881c": "^2.0.0",
    }
    errors = []
    for dependency, version in expected.items():
        if str(dependencies.get(dependency)) != version:
            errors.append(f"{manifest_path.relative_to(REPO)}: {dependency} version must be {version!r}")
    return errors


JD9365_IDF6_CF = "JD9365_800_1280_PANEL_60HZ_DPI_CONFIG_CF"
JD9365_LEGACY = "JD9365_800_1280_PANEL_60HZ_DPI_CONFIG"
JD9365_ADAPTER = f"""#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#if defined({JD9365_IDF6_CF})
#define BSP_JD9365_800_1280_PANEL_DPI_CONFIG(_color_format) {JD9365_IDF6_CF}(_color_format)
#else
#define BSP_JD9365_800_1280_PANEL_DPI_CONFIG(_color_format) {JD9365_LEGACY}(_color_format)
#endif
#else
#define BSP_JD9365_800_1280_PANEL_DPI_CONFIG(_color_format) {JD9365_LEGACY}(_color_format)
#endif"""
JD9365_IDF6_FALLBACK = f"""#else
#define BSP_JD9365_800_1280_PANEL_DPI_CONFIG(_color_format) {JD9365_LEGACY}(_color_format)
#endif"""
JD9365_COLOR_SELECTION = """#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    esp_lcd_dpi_panel_config_t dpi_config = BSP_JD9365_800_1280_PANEL_DPI_CONFIG(LCD_COLOR_FMT_RGB888);
#else
    esp_lcd_dpi_panel_config_t dpi_config = BSP_JD9365_800_1280_PANEL_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#endif
#else
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    esp_lcd_dpi_panel_config_t dpi_config = BSP_JD9365_800_1280_PANEL_DPI_CONFIG(LCD_COLOR_FMT_RGB565);
#else
    esp_lcd_dpi_panel_config_t dpi_config = BSP_JD9365_800_1280_PANEL_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#endif
#endif"""
ILI9881C_COLOR_SELECTION = """#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
        .in_color_format = LCD_COLOR_FMT_RGB888,
#else
        .in_color_format = LCD_COLOR_FMT_RGB565,
#endif
#elif CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB888,
#else
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
#endif"""


def p4_lcd_x_display_source_errors(source):
    errors = []
    if JD9365_ADAPTER not in source:
        errors.append("JD9365 must feature-detect the IDF 6 _CF API, fall back to the Waveshare API, and retain the IDF 5 legacy API")
    if JD9365_COLOR_SELECTION not in source:
        errors.append("JD9365 must pass LCD_COLOR_FMT_* on IDF 6 and LCD_COLOR_PIXEL_FORMAT_* on IDF 5")
    if ILI9881C_COLOR_SELECTION not in source:
        errors.append("ILI9881C must use IDF 6 in_color_format and IDF 5 pixel_format")
    return errors


def check_p4_lcd_x_display_source_compatibility(components):
    component_dir = "bsp/esp32_p4_wifi6_touch_lcd_x"
    if component_dir not in components:
        return []

    source_path = REPO / component_dir / "esp32_p4_wifi6_touch_lcd_x.c"
    return [f"{source_path.relative_to(REPO)}: {error}" for error in p4_lcd_x_display_source_errors(source_path.read_text(encoding="utf-8"))]


def run_synthetic_tests():
    valid_source = "\n".join((JD9365_ADAPTER, JD9365_COLOR_SELECTION, ILI9881C_COLOR_SELECTION))
    assert not p4_lcd_x_display_source_errors(valid_source), "synthetic JD9365/ILI9881C compatibility source must pass"
    assert p4_lcd_x_display_source_errors(valid_source.replace(JD9365_IDF6_FALLBACK, "", 1)), "missing JD9365 IDF 6 fallback must fail"
    assert p4_lcd_x_display_source_errors(valid_source.replace(f"{JD9365_IDF6_CF}(_color_format)", "missing_cf(_color_format)", 1)), "missing JD9365 _CF path must fail"
    assert not check_p4_lcd_x_display_source_compatibility(set()), "non-target components must not be checked"
    print("Synthetic compatibility checks passed")


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
    components, global_files = split_changed_files(files, component_dirs)
    output_components = build_components(components, component_dirs, global_files)

    unlisted = sorted(set(components) - upload_dirs)
    output_unlisted = sorted(set(output_components) - upload_dirs)
    errors = check_lcd5_hx8394_contract(upload_dirs)
    if unlisted:
        errors.extend(f"{item}: component is not listed in upload_component.yml" for item in unlisted)
    if output_unlisted:
        errors.extend(f"{item}: build component is not listed in upload_component.yml" for item in output_unlisted)

    errors.extend(check_version_bumps(base_ref, components))
    errors.extend(check_bsp_codec_dependency(output_components))
    errors.extend(check_p4_lcd_x_panel_dependencies(output_components))
    errors.extend(check_p4_lcd_x_display_source_compatibility(components))
    write_outputs(output_components)

    print(f"Base ref: {base_ref}")
    print(f"Changed files: {len(files)}")
    print(f"Changed components: {len(components)}")
    for component in components:
        print(f"  - {component}")
    print(f"Global changed files: {len(global_files)}")
    for file_name in global_files:
        print(f"  - {file_name}")
    print(f"Build components: {len(output_components)}")
    for component in output_components:
        print(f"  - {component}")

    if errors:
        print("\nManifest check failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    if sys.argv[1:] == ["--synthetic-tests"]:
        run_synthetic_tests()
    else:
        main()
