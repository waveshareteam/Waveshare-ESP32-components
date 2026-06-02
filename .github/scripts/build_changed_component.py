#!/usr/bin/env python3
import os
import shutil
import subprocess
import sys
from pathlib import Path

import yaml


REPO = Path.cwd()
DEFAULT_TARGET = "esp32s3"


def load_manifest(path):
    with open(path, "r", encoding="utf-8") as manifest_file:
        return yaml.safe_load(manifest_file) or {}


def component_targets(component_dir):
    manifest = load_manifest(component_dir / "idf_component.yml")
    targets = manifest.get("targets") or [DEFAULT_TARGET]
    if isinstance(targets, str):
        targets = [targets]
    return [str(target) for target in targets]


def candidate_projects(component_dir):
    projects = []
    for name in ("test_app", "test_apps"):
        candidate = component_dir / name
        if (candidate / "CMakeLists.txt").exists():
            projects.append(candidate)
    return projects


def write_generated_project(component_dir):
    component_name = component_dir.name
    project_dir = REPO / ".component_self_check" / component_dir.as_posix().replace("/", "__")
    if project_dir.exists():
        shutil.rmtree(project_dir)
    (project_dir / "main").mkdir(parents=True)

    (project_dir / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.16)\n"
        "include($ENV{IDF_PATH}/tools/cmake/project.cmake)\n"
        "project(component_self_check)\n",
        encoding="utf-8",
    )
    (project_dir / "main" / "CMakeLists.txt").write_text(
        'idf_component_register(SRCS "main.c")\n',
        encoding="utf-8",
    )
    (project_dir / "main" / "main.c").write_text(
        "void app_main(void)\n"
        "{\n"
        "}\n",
        encoding="utf-8",
    )
    (project_dir / "main" / "idf_component.yml").write_text(
        "dependencies:\n"
        f"  {component_name}:\n"
        '    version: "*"\n'
        f'    override_path: "{component_dir.resolve().as_posix()}"\n',
        encoding="utf-8",
    )
    return project_dir


def run_build(project_dir, target, build_root):
    build_dir = build_root / project_dir.name / target
    command = [
        "idf.py",
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
        projects = [write_generated_project(component_dir)]

    build_root = REPO / ".component_self_check_build"
    for target in component_targets(component_dir):
        for project in projects:
            run_build(project, target, build_root)


if __name__ == "__main__":
    main()
