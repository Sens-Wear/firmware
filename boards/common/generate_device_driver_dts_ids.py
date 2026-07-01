#!/usr/bin/env python3
"""Generate device ID macros from the merged Zephyr DTS.

The output is a small header with one macro per labeled node, using the final
merged DTS node order to assign stable build-time IDs.

Every device driver gets an ID. Device nodes are matched at any nesting depth
(for example a sensor sitting on an I2C/SPI bus controller, or a regulator under
a PMIC), independent of whether the device has interrupt pins: the only criteria
are that the node is labeled, carries a ``compatible`` property, and is not
``status = "disabled"``.
"""

from __future__ import annotations

import argparse
import pathlib
import re
from dataclasses import dataclass

NODE_START_RE = re.compile(
    r"^\s*([A-Za-z_][A-Za-z0-9_]*):\s*[^{}]+\{\s*(?:/\*.*\*/\s*)?$")
# A node's ``compatible`` may list several strings, which the DTS pretty-printer
# wraps across multiple lines (e.g. `compatible = "a",` / `             "b";`).
# Only the first value on the `compatible =` line is needed: the IDs are keyed off
# the node label, so this match exists purely to confirm the node is a device.
COMPATIBLE_RE = re.compile(r'^\s*compatible\s*=\s*"([^"]+)"')
STATUS_RE = re.compile(r'^\s*status\s*=\s*"([^"]+)";\s*(?:/\*.*\*/\s*)?$')


@dataclass
class Node:
    label: str
    order: int
    compatible: str | None = None
    status: str | None = None

    @property
    def macro_name(self) -> str:
        return f"{normalize_label(self.label)}_DEVICE_DTS_ID"


def normalize_label(label: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]", "_", label).upper()


def parse_dts(path: pathlib.Path) -> list[Node]:
    """Collect every labeled device node, descending into nested children.

    A stack of open nodes is maintained so that a device on a bus (or any other
    nested node) is detected just like a top-level node. Properties are attached
    to the innermost open node, and a node qualifies for an ID when it has a
    ``compatible`` property and is not explicitly disabled.
    """
    qualifying: list[Node] = []
    stack: list[Node] = []
    order = 0

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        start_match = NODE_START_RE.match(raw_line)
        if start_match is not None:
            # A node-start line ends with the opening brace, so it contributes
            # exactly one level of nesting and carries no properties itself.
            stack.append(Node(label=start_match.group(1), order=order))
            order += 1
            continue

        if stack:
            compatible_match = COMPATIBLE_RE.match(raw_line)
            if compatible_match is not None:
                stack[-1].compatible = compatible_match.group(1)

            status_match = STATUS_RE.match(raw_line)
            if status_match is not None:
                stack[-1].status = status_match.group(1)

        # Closing braces pop the innermost open node(s). A line may close more
        # than one level (for example "}; };") or none at all.
        closes = raw_line.count("}") - raw_line.count("{")
        for _ in range(closes):
            if not stack:
                break
            node = stack.pop()
            if node.compatible is not None and node.status != "disabled":
                qualifying.append(node)

    # Assign IDs in declaration order rather than the order nodes close.
    qualifying.sort(key=lambda node: node.order)
    return qualifying


def write_header(
        nodes: list[Node], output_path: pathlib.Path, dts_path: pathlib.Path) -> None:
    lines = [
        "/* Generated from the merged Zephyr DTS. */",
        "#ifndef SENSWEAR_GENERATED_DEVICE_DRIVER_IDS_H_",
        "#define SENSWEAR_GENERATED_DEVICE_DRIVER_IDS_H_",
        "",
        "#define DEVICE_ID_INVALID 0u",
        "",
    ]

    for index, node in enumerate(nodes, start=1):
        lines.append(f"#define {node.macro_name} ({index}u)")

    lines.extend(
        [
            "",
            f"#define SENSEWEAR_GENERATED_DEVICE_COUNT ({len(nodes)}u)",
            "",
            f"/* Source DTS: {dts_path.as_posix()} */",
            "",
            "#endif /* SENSWEAR_GENERATED_DEVICE_DRIVER_IDS_H_ */",
            "",
        ]
    )

    output_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        required=True,
        help="Path to the merged zephyr.dts")
    parser.add_argument(
        "--output",
        required=True,
        help="Path to the generated header")
    args = parser.parse_args()

    dts_path = pathlib.Path(args.input)
    output_path = pathlib.Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if not dts_path.exists():
        raise SystemExit(f"DTS file not found: {dts_path}")

    nodes = parse_dts(dts_path)
    write_header(nodes, output_path, dts_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
