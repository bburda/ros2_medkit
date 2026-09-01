#!/bin/bash
# Copyright 2026 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Release helper script for ros2_medkit.
#
# Usage:
#   ./scripts/release.sh bump <version>    - Bump all package.xml and version.hpp to <version>
#   ./scripts/release.sh verify [<version>] - Verify all packages have consistent versions
#                                             (if <version> given, checks against that specific version)
#
# Examples:
#   ./scripts/release.sh bump 0.4.0
#   ./scripts/release.sh verify
#   ./scripts/release.sh verify 0.4.0

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_DIR="${REPO_ROOT}/src"
VERSION_HPP="${SRC_DIR}/ros2_medkit_gateway/include/ros2_medkit_gateway/core/version.hpp"
CONF_PY="${REPO_ROOT}/docs/conf.py"
DOCS_PYPROJECT="${REPO_ROOT}/docs/pyproject.toml"
DOXYFILE="${REPO_ROOT}/docs/Doxyfile"
QUALITY_DECL="${REPO_ROOT}/QUALITY_DECLARATION.md"
REST_RST="${REPO_ROOT}/docs/api/rest.rst"

# Read one assignment out of docs/conf.py. Accepts either quote style and any
# spacing around "="; the value is anchored to the closing quote so a suffixed
# version is returned whole rather than truncated to its numeric prefix.
conf_py_field() {
    grep -oP "^[[:space:]]*$1[[:space:]]*=[[:space:]]*[\"']\\K[^\"']*(?=[\"'][[:space:]]*\$)" "$CONF_PY" | head -1
}

usage() {
    echo "Usage: $0 {bump <version>|verify [<version>]}"
    echo ""
    echo "Commands:"
    echo "  bump <version>     Bump all package.xml files and version.hpp fallback"
    echo "  verify [<version>] Verify version consistency across all packages"
    exit 1
}

find_package_xmls() {
    find "${SRC_DIR}" -name "package.xml" -not -path "*/.worktrees/*" -not -path "*/build/*" -not -path "*/install/*" | sort
}

get_version() {
    local pkg_xml="$1"
    grep -oP '<version>\K[0-9]+\.[0-9]+\.[0-9]+(?=</version>)' "$pkg_xml"
}

get_package_name() {
    local pkg_xml="$1"
    grep -oP '<name>\K[^<]+' "$pkg_xml"
}

cmd_bump() {
    local target_version="$1"

    # Validate semver format
    if ! echo "$target_version" | grep -qP '^[0-9]+\.[0-9]+\.[0-9]+$'; then
        echo "Error: '$target_version' is not a valid semver (X.Y.Z)"
        exit 1
    fi

    echo "Bumping all packages to version ${target_version}..."
    echo ""

    local count=0
    while IFS= read -r pkg_xml; do
        local pkg_name
        pkg_name=$(get_package_name "$pkg_xml")
        local old_version
        old_version=$(get_version "$pkg_xml")
        local rel_path="${pkg_xml#"${REPO_ROOT}/"}"

        sed -i "s|<version>[0-9]\+\.[0-9]\+\.[0-9]\+</version>|<version>${target_version}</version>|" "$pkg_xml"
        echo "  ${pkg_name}: ${old_version} -> ${target_version}  (${rel_path})"
        count=$((count + 1))
    done < <(find_package_xmls)

    # Update version.hpp fallback
    if [ -f "$VERSION_HPP" ]; then
        local old_fallback
        old_fallback=$(grep -oP 'kGatewayVersion = "\K[0-9]+\.[0-9]+\.[0-9]+' "$VERSION_HPP" || echo "unknown")
        sed -i "s|kGatewayVersion = \"[0-9]\+\.[0-9]\+\.[0-9]\+\"|kGatewayVersion = \"${target_version}\"|" "$VERSION_HPP"
        echo "  version.hpp fallback: ${old_fallback} -> ${target_version}"
    else
        echo "  WARNING: version.hpp not found at ${VERSION_HPP}"
    fi

    # Update docs/conf.py version and release
    if [ -f "$CONF_PY" ]; then
        local old_conf
        old_conf=$(conf_py_field version || echo "unknown")
        sed -i -E "s|^([[:space:]]*version[[:space:]]*=[[:space:]]*)[\"'][^\"']*[\"']|\\1'${target_version}'|" "$CONF_PY"
        sed -i -E "s|^([[:space:]]*release[[:space:]]*=[[:space:]]*)[\"'][^\"']*[\"']|\\1'${target_version}'|" "$CONF_PY"
        # sed exits 0 whether or not it substituted, so confirm the rewrite landed
        # rather than reporting a bump the file never received.
        if [ "$(conf_py_field version)" != "${target_version}" ] || [ "$(conf_py_field release)" != "${target_version}" ]; then
            echo "  ERROR: docs/conf.py was not rewritten to ${target_version}" >&2
            return 1
        fi
        echo "  docs/conf.py: ${old_conf} -> ${target_version}"
    fi

    # Update docs/pyproject.toml version
    if [ -f "$DOCS_PYPROJECT" ]; then
        sed -i "s|^version = \"[0-9]\+\.[0-9]\+\.[0-9]\+\"|version = \"${target_version}\"|" "$DOCS_PYPROJECT"
        echo "  docs/pyproject.toml: -> ${target_version}"
    fi

    # Update docs/Doxyfile PROJECT_NUMBER
    if [ -f "$DOXYFILE" ]; then
        sed -i "s|\(PROJECT_NUMBER[[:space:]]*=[[:space:]]*\)\"[0-9]\+\.[0-9]\+\.[0-9]\+\"|\1\"${target_version}\"|" "$DOXYFILE"
        echo "  docs/Doxyfile PROJECT_NUMBER: -> ${target_version}"
    fi

    # Update QUALITY_DECLARATION.md current-version references (leave the
    # ">=1.0.0" stability-policy mentions untouched via anchored patterns)
    if [ -f "$QUALITY_DECL" ]; then
        sed -i \
            -e "s|current version is \*\*[0-9]\+\.[0-9]\+\.[0-9]\+\*\*|current version is **${target_version}**|" \
            -e "s|all packages at [0-9]\+\.[0-9]\+\.[0-9]\+|all packages at ${target_version}|" \
            -e "s|Version is [0-9]\+\.[0-9]\+\.[0-9]\+|Version is ${target_version}|" \
            "$QUALITY_DECL"
        echo "  QUALITY_DECLARATION.md: -> ${target_version}"
    fi

    # Update the gateway-version literals in docs/api/rest.rst example responses.
    # Each is anchored on its adjacent "name" key so the SOVD API "version":
    # "1.0.0" (which has neither anchor) is never rewritten:
    #   - root "/" response:       "name": "ROS 2 Medkit Gateway" then "version"
    #   - version-info vendor_info: "version" then "name": "ros2_medkit"
    if [ -f "$REST_RST" ]; then
        REST_TARGET="$target_version" perl -0pi -e '
            my $v = $ENV{REST_TARGET};
            s/("name": "ROS 2 Medkit Gateway",\s*\n\s*"version": ")\d+\.\d+\.\d+(")/$1$v$2/g;
            s/("version": ")\d+\.\d+\.\d+(",\s*\n\s*"name": "ros2_medkit")/$1$v$2/g;
        ' "$REST_RST"
        echo "  docs/api/rest.rst example versions: -> ${target_version}"
    fi

    echo ""
    echo "Bumped ${count} packages + version.hpp + docs to ${target_version}."
    echo ""
    echo "Run '$0 verify ${target_version}' to confirm."
}

cmd_verify() {
    local expected_version="${1:-}"

    echo "Verifying package versions..."
    echo ""

    local all_ok=true
    local versions_seen=()

    while IFS= read -r pkg_xml; do
        local pkg_name
        pkg_name=$(get_package_name "$pkg_xml")
        local version
        version=$(get_version "$pkg_xml")
        local rel_path="${pkg_xml#"${REPO_ROOT}/"}"

        if [ -n "$expected_version" ] && [ "$version" != "$expected_version" ]; then
            echo "  MISMATCH: ${pkg_name} is ${version}, expected ${expected_version}  (${rel_path})"
            all_ok=false
        else
            echo "  OK: ${pkg_name} = ${version}  (${rel_path})"
        fi
        versions_seen+=("$version")
    done < <(find_package_xmls)

    # Check version.hpp fallback
    if [ -f "$VERSION_HPP" ]; then
        local hpp_version
        hpp_version=$(grep -oP 'kGatewayVersion = "\K[0-9]+\.[0-9]+\.[0-9]+' "$VERSION_HPP" || echo "unknown")
        if [ -n "$expected_version" ] && [ "$hpp_version" != "$expected_version" ]; then
            echo "  MISMATCH: version.hpp fallback is ${hpp_version}, expected ${expected_version}"
            all_ok=false
        else
            echo "  OK: version.hpp fallback = ${hpp_version}"
        fi
        versions_seen+=("$hpp_version")
    fi

    # Check docs/conf.py
    if [ -f "$CONF_PY" ]; then
        local conf_version conf_release
        conf_version=$(conf_py_field version || echo "unknown")
        conf_release=$(conf_py_field release || echo "unknown")
        if [ -n "$expected_version" ] && [ "$conf_version" != "$expected_version" ]; then
            echo "  MISMATCH: docs/conf.py version is ${conf_version}, expected ${expected_version}"
            all_ok=false
        elif [ "$conf_release" != "$conf_version" ]; then
            echo "  MISMATCH: docs/conf.py release is ${conf_release}, version is ${conf_version}"
            all_ok=false
        else
            echo "  OK: docs/conf.py = ${conf_version}"
        fi
        versions_seen+=("$conf_version")
    fi

    # Check docs/pyproject.toml
    if [ -f "$DOCS_PYPROJECT" ]; then
        local pyproject_version
        pyproject_version=$(grep -oP '^version = "\K[0-9]+\.[0-9]+\.[0-9]+' "$DOCS_PYPROJECT" || echo "unknown")
        if [ -n "$expected_version" ] && [ "$pyproject_version" != "$expected_version" ]; then
            echo "  MISMATCH: docs/pyproject.toml is ${pyproject_version}, expected ${expected_version}"
            all_ok=false
        else
            echo "  OK: docs/pyproject.toml = ${pyproject_version}"
        fi
        versions_seen+=("$pyproject_version")
    fi

    # Check docs/Doxyfile PROJECT_NUMBER
    if [ -f "$DOXYFILE" ]; then
        local doxy_version
        doxy_version=$(grep -oP 'PROJECT_NUMBER\s*=\s*"\K[0-9]+\.[0-9]+\.[0-9]+' "$DOXYFILE" || echo "unknown")
        if [ -n "$expected_version" ] && [ "$doxy_version" != "$expected_version" ]; then
            echo "  MISMATCH: docs/Doxyfile PROJECT_NUMBER is ${doxy_version}, expected ${expected_version}"
            all_ok=false
        else
            echo "  OK: docs/Doxyfile PROJECT_NUMBER = ${doxy_version}"
        fi
        versions_seen+=("$doxy_version")
    fi

    # Check QUALITY_DECLARATION.md current version
    if [ -f "$QUALITY_DECL" ]; then
        local qd_version
        qd_version=$(grep -oP 'current version is \*\*\K[0-9]+\.[0-9]+\.[0-9]+' "$QUALITY_DECL" || echo "unknown")
        if [ -n "$expected_version" ] && [ "$qd_version" != "$expected_version" ]; then
            echo "  MISMATCH: QUALITY_DECLARATION.md current version is ${qd_version}, expected ${expected_version}"
            all_ok=false
        else
            echo "  OK: QUALITY_DECLARATION.md current version = ${qd_version}"
        fi
        versions_seen+=("$qd_version")
    fi

    # Check docs/api/rest.rst example-response versions (gateway version only;
    # the SOVD API "version": "1.0.0" is excluded via the same name anchors as
    # the bump path, so it is neither rewritten nor verified here).
    if [ -f "$REST_RST" ]; then
        local rest_versions
        rest_versions=$(perl -0ne '
            while (/"name": "ROS 2 Medkit Gateway",\s*\n\s*"version": "(\d+\.\d+\.\d+)"/g) { print "$1\n" }
            while (/"version": "(\d+\.\d+\.\d+)",\s*\n\s*"name": "ros2_medkit"/g) { print "$1\n" }
        ' "$REST_RST")
        [ -n "$rest_versions" ] || rest_versions="unknown"
        while IFS= read -r rest_version; do
            if [ -n "$expected_version" ] && [ "$rest_version" != "$expected_version" ]; then
                echo "  MISMATCH: docs/api/rest.rst example version is ${rest_version}, expected ${expected_version}"
                all_ok=false
            else
                echo "  OK: docs/api/rest.rst example version = ${rest_version}"
            fi
            versions_seen+=("$rest_version")
        done <<< "$rest_versions"
    fi

    # Check consistency if no expected version given
    if [ -z "$expected_version" ]; then
        # An "unknown" means a version pattern failed to match in some file.
        # Without this guard an all-"unknown" run collapses to one unique value
        # and would false-pass the consistency check below.
        if printf '%s\n' "${versions_seen[@]}" | grep -qx "unknown"; then
            echo ""
            echo "ERROR: could not parse a version from one or more files (got 'unknown')."
            all_ok=false
        fi

        local unique_versions
        unique_versions=$(printf '%s\n' "${versions_seen[@]}" | sort -u)
        local unique_count
        unique_count=$(echo "$unique_versions" | wc -l)
        if [ "$unique_count" -gt 1 ]; then
            echo ""
            echo "WARNING: Found multiple versions:"
            echo "$unique_versions" | while read -r v; do echo "  - $v"; done
            all_ok=false
        fi
    fi

    echo ""
    if $all_ok; then
        echo "All versions are consistent."
        return 0
    else
        echo "Version mismatches found!"
        return 1
    fi
}

# Main
if [ $# -lt 1 ]; then
    usage
fi

case "$1" in
    bump)
        if [ $# -lt 2 ]; then
            echo "Error: bump requires a version argument"
            usage
        fi
        cmd_bump "$2"
        ;;
    verify)
        cmd_verify "${2:-}"
        ;;
    *)
        usage
        ;;
esac
