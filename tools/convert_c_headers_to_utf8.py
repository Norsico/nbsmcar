#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


TEXT_SUFFIXES = {".c", ".h"}
SKIP_DIRS = {".git", ".hg", ".svn", "__pycache__"}
FALLBACK_ENCODINGS = ("utf-8", "gb18030", "gbk", "gb2312")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert all .c and .h files under a directory to UTF-8."
    )
    parser.add_argument(
        "root",
        nargs="?",
        default=".",
        help="Root directory to scan. Defaults to the current directory.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Report what would change without writing files.",
    )
    parser.add_argument(
        "--line-endings",
        choices=("preserve", "lf", "crlf"),
        default="preserve",
        help="Output line endings. Defaults to preserve.",
    )
    parser.add_argument(
        "--backup-ext",
        default="",
        help="Optional backup suffix, for example '.bak'.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print unchanged UTF-8 files too.",
    )
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="Only print the final summary.",
    )
    parser.add_argument(
        "--allow-latin1",
        action="store_true",
        help="Try latin-1 as a last-resort fallback. Use only when you know files are single-byte encoded.",
    )
    return parser.parse_args()


def iter_source_files(root: Path):
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        if any(part in SKIP_DIRS for part in path.parts):
            continue
        if path.suffix.lower() in TEXT_SUFFIXES:
            yield path


def detect_encoding(data: bytes) -> tuple[str | None, str | None]:
    if not data:
        return "utf-8", ""

    if b"\x00" in data:
        return None, None

    if data.startswith(b"\xef\xbb\xbf"):
        try:
            return "utf-8-sig", data.decode("utf-8-sig")
        except UnicodeDecodeError:
            pass

    for encoding in FALLBACK_ENCODINGS:
        try:
            return encoding, data.decode(encoding)
        except UnicodeDecodeError:
            continue

    try:
        return "latin-1", data.decode("latin-1")
    except UnicodeDecodeError:
        return None, None


def detect_newline(data: bytes) -> str:
    crlf = data.count(b"\r\n")
    lf = data.count(b"\n") - crlf
    cr = data.count(b"\r") - crlf

    if crlf >= lf and crlf >= cr and crlf > 0:
        return "\r\n"
    if lf >= cr and lf > 0:
        return "\n"
    if cr > 0:
        return "\r"
    return "\n"


def normalize_text(text: str, newline: str) -> str:
    normalized = text.replace("\r\n", "\n").replace("\r", "\n")
    if newline == "\n":
        return normalized
    return normalized.replace("\n", newline)


def choose_newline(arg_value: str, original: str) -> str:
    if arg_value == "preserve":
        return original
    if arg_value == "lf":
        return "\n"
    return "\r\n"


def make_backup(path: Path, data: bytes, backup_ext: str) -> None:
    if not backup_ext:
        return
    backup_path = path.with_name(path.name + backup_ext)
    if backup_path.exists():
        return
    backup_path.write_bytes(data)


def convert_file(path: Path, args: argparse.Namespace) -> tuple[str, str]:
    original_bytes = path.read_bytes()
    encoding, text = detect_encoding(original_bytes)
    if encoding is None or text is None:
        return "skipped", "undecodable or binary-looking"

    if encoding == "latin-1" and not args.allow_latin1:
        return "skipped", "latin-1 fallback disabled"

    original_newline = detect_newline(original_bytes)
    output_newline = choose_newline(args.line_endings, original_newline)
    normalized_text = normalize_text(text, output_newline)
    utf8_bytes = normalized_text.encode("utf-8")

    if original_bytes == utf8_bytes:
        return "unchanged", encoding

    if not args.dry_run:
        make_backup(path, original_bytes, args.backup_ext)
        path.write_bytes(utf8_bytes)

    return "converted", encoding


def main() -> int:
    args = parse_args()
    root = Path(args.root).resolve()

    if not root.exists():
        print(f"error: root does not exist: {root}", file=sys.stderr)
        return 2

    converted = 0
    unchanged = 0
    skipped = 0

    for path in iter_source_files(root):
        status, detail = convert_file(path, args)
        rel = path.relative_to(root)

        if status == "converted":
            converted += 1
            if not args.summary_only:
                action = "would convert" if args.dry_run else "converted"
                print(f"{action}: {rel} [{detail} -> utf-8]")
        elif status == "unchanged":
            unchanged += 1
            if args.verbose and not args.summary_only:
                print(f"unchanged: {rel} [{detail}]")
        else:
            skipped += 1
            if not args.summary_only:
                print(f"skipped: {rel} [{detail}]")

    mode = "dry-run" if args.dry_run else "write"
    print(
        f"done: mode={mode} converted={converted} unchanged={unchanged} skipped={skipped}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
