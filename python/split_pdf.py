#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from pypdf import PdfReader, PdfWriter


def write_range(reader: PdfReader, start: int, end: int, out_path: Path) -> None:
    """Write pages [start, end) using 0-based indexing."""
    writer = PdfWriter()
    for i in range(start, end):
        writer.add_page(reader.pages[i])
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("wb") as f:
        writer.write(f)


def split_pdf_at_page(input_pdf: Path, split_after_page: int) -> tuple[Path, Path]:
    """
    split_after_page is 1-based:
      - Part A: pages 1..split_after_page
      - Part B: pages split_after_page+1..end
    """
    reader = PdfReader(str(input_pdf))
    n = len(reader.pages)
    k = split_after_page

    if not (1 <= k < n):
        raise ValueError(f"--split-after must be between 1 and {n-1}, got {k} (PDF has {n} pages).")

    out_a = input_pdf.with_name(f"{input_pdf.stem}_p1-{k}{input_pdf.suffix}")
    out_b = input_pdf.with_name(f"{input_pdf.stem}_p{k+1}-{n}{input_pdf.suffix}")

    write_range(reader, 0, k, out_a)   # 1..k
    write_range(reader, k, n, out_b)   # k+1..n

    return out_a, out_b


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Split a PDF into two parts after a given page (1-based)."
    )
    p.add_argument(
        "--input", "-i",
        required=True,
        help="Path to the input PDF (Windows path is fine).",
    )
    p.add_argument(
        "--split-after", "-s",
        required=True,
        type=int,
        help="1-based page number to split after (Part A ends at this page).",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    input_pdf = Path(args.input)

    if not input_pdf.exists():
        raise FileNotFoundError(f"Input not found: {input_pdf}")
    if input_pdf.suffix.lower() != ".pdf":
        raise ValueError(f"Input must be a .pdf file: {input_pdf}")

    out_a, out_b = split_pdf_at_page(input_pdf, args.split_after)
    print(f"Done.\n  A -> {out_a}\n  B -> {out_b}")


if __name__ == "__main__":
    main()
