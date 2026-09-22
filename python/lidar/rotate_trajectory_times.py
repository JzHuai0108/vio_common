#!/usr/bin/env python3
"""Apply a linearly varying timestamp shift from a selected epoch onward.

Given input rows ``(t_i, p_i)``, an epoch, a starting shift ``s_0``, and an
ending shift ``s_1``, this script writes

``(t_i, p_i)`` for rows before the epoch, and
``(t_i + s(t_i), p_i)`` from the epoch row onward, where ``s(t)`` varies
linearly from ``s_0`` at the selected epoch row to ``s_1`` at the final row.

An epoch of zero selects every trajectory row. Any other epoch is matched to
the closest input timestamp within 1 ms by default.
Pose and all other non-timestamp fields are copied without modification.
"""

import argparse
import os
import stat
import tempfile
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import List, Sequence, Tuple


LineRecord = Tuple[int, str, str]


def load_trajectory(path: Path) -> Tuple[List[str], List[LineRecord], List[Decimal]]:
    """Return original lines, replaceable timestamp pieces, and timestamps."""
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    records: List[LineRecord] = []
    timestamps: List[Decimal] = []

    for line_index, line in enumerate(lines):
        stripped = line.lstrip()
        if not stripped or stripped.startswith("#"):
            continue

        leading_length = len(line) - len(stripped)
        token_end = 0
        while token_end < len(stripped) and not stripped[token_end].isspace():
            token_end += 1
        timestamp_token = stripped[:token_end]
        if not timestamp_token:
            raise ValueError(f"Missing timestamp at line {line_index + 1}")
        try:
            timestamp = Decimal(timestamp_token)
        except InvalidOperation as exc:
            raise ValueError(
                f"Invalid timestamp {timestamp_token!r} at line {line_index + 1}"
            ) from exc

        # Preserve leading whitespace and everything following column 1.
        records.append(
            (line_index, line[:leading_length], stripped[token_end:])
        )
        timestamps.append(timestamp)

    if not records:
        raise ValueError(f"Trajectory contains no data rows: {path}")
    for i in range(1, len(timestamps)):
        if timestamps[i] <= timestamps[i - 1]:
            raise ValueError(
                "Input timestamps must be strictly increasing; "
                f"rows {i - 1} and {i} violate this"
            )
    return lines, records, timestamps


def decimal_places(token: str) -> int:
    """Return the number of fixed-point digits used by a timestamp token."""
    token = token.lower()
    if "e" in token:
        return max(0, -Decimal(token).as_tuple().exponent)
    if "." not in token:
        return 0
    return len(token.rsplit(".", 1)[1])


def find_epoch_row(
    timestamps: Sequence[Decimal],
    epoch: Decimal,
    tolerance: Decimal,
) -> Tuple[int, Decimal]:
    """Return the closest timestamp row and its signed offset from epoch."""
    if tolerance < 0:
        raise ValueError("epoch tolerance must be nonnegative")
    row = min(
        range(len(timestamps)),
        key=lambda index: abs(timestamps[index] - epoch),
    )
    difference = timestamps[row] - epoch
    if abs(difference) > tolerance:
        raise ValueError(
            f"No trajectory timestamp is within {tolerance} s of epoch "
            f"{epoch}; closest is row {row}, timestamp {timestamps[row]} "
            f"(difference {difference:+} s)"
        )
    return row, difference


def shifted_timestamp_tokens(
    timestamps: Sequence[Decimal],
    original_tokens: Sequence[str],
    start_row: int,
    start_shift: Decimal,
    end_shift: Decimal,
) -> List[str]:
    """Linearly shift times at and after start_row, retaining pose rows."""
    if not 0 <= start_row < len(timestamps):
        raise ValueError("start_row is outside the trajectory")
    precision = max(
        max(decimal_places(token) for token in original_tokens),
        max(0, -start_shift.as_tuple().exponent),
        max(0, -end_shift.as_tuple().exponent),
    )
    output: List[str] = list(original_tokens[:start_row])
    shift_duration = timestamps[-1] - timestamps[start_row]
    for row_index in range(start_row, len(timestamps)):
        if shift_duration == 0:
            shift = start_shift
        else:
            fraction = (
                timestamps[row_index] - timestamps[start_row]
            ) / shift_duration
            shift = start_shift + fraction * (end_shift - start_shift)
        shifted = timestamps[row_index] + shift
        output.append(f"{shifted:.{precision}f}")

    shifted_timestamps = [Decimal(token) for token in output]
    for row_index in range(1, len(shifted_timestamps)):
        if shifted_timestamps[row_index] <= shifted_timestamps[row_index - 1]:
            raise ValueError(
                "Shifted timestamps must be strictly increasing; "
                f"rows {row_index - 1} and {row_index} would violate this"
            )
    return output


def write_shifted_trajectory(
    input_path: Path,
    output_path: Path,
    lines: Sequence[str],
    records: Sequence[LineRecord],
    timestamp_tokens: Sequence[str],
    overwrite: bool,
) -> None:
    if input_path.resolve() == output_path.resolve():
        raise ValueError("Refusing to overwrite the input trajectory in place")
    if output_path.exists() and not overwrite:
        raise FileExistsError(
            f"Output already exists: {output_path}; pass --overwrite to replace it"
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    output_lines = list(lines)
    for (line_index, leading, remainder), timestamp in zip(
        records, timestamp_tokens
    ):
        output_lines[line_index] = leading + timestamp + remainder

    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output_path.name}.", suffix=".tmp", dir=output_path.parent
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8") as stream:
            stream.writelines(output_lines)
        os.chmod(temporary_path, stat.S_IMODE(input_path.stat().st_mode))
        os.replace(temporary_path, output_path)
    except BaseException:
        try:
            temporary_path.unlink()
        except FileNotFoundError:
            pass
        raise


def default_output_path(input_path: Path, output_dir: Path) -> Path:
    return output_dir / f"{input_path.stem}_time_shifted{input_path.suffix}"


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "From the trajectory row nearest EPOCH onward, add a shift that "
            "varies linearly from START_SHIFT to END_SHIFT."
        )
    )
    parser.add_argument("input", type=Path, help="input trajectory text file")
    parser.add_argument(
        "epoch",
        type=Decimal,
        help=(
            "input timestamp at which to begin shifting (inclusive); "
            "use 0 to shift every row"
        ),
    )
    parser.add_argument(
        "start_shift",
        type=Decimal,
        help="seconds added at the selected epoch row",
    )
    parser.add_argument(
        "end_shift",
        type=Decimal,
        help="seconds added at the final trajectory row",
    )
    parser.add_argument(
        "--epoch-tolerance",
        type=Decimal,
        default=Decimal("0.001"),
        help="maximum epoch matching error in seconds (default: 0.001)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="output directory (default: input file directory)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="explicit output path",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="replace existing output files",
    )
    args = parser.parse_args()

    input_path = args.input.expanduser().resolve()
    if not input_path.is_file():
        parser.error(f"input trajectory does not exist: {input_path}")
    if args.epoch_tolerance < 0:
        parser.error("--epoch-tolerance must be nonnegative")

    lines, records, timestamps = load_trajectory(input_path)
    original_tokens = []
    for line_index, leading, _ in records:
        stripped = lines[line_index][len(leading):]
        original_tokens.append(stripped.split(None, 1)[0])

    if args.epoch == 0:
        start_row = 0
        epoch_difference = None
    else:
        try:
            start_row, epoch_difference = find_epoch_row(
                timestamps, args.epoch, args.epoch_tolerance
            )
        except ValueError as exc:
            parser.error(str(exc))

    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir is not None
        else input_path.parent
    )
    output_path = (
        args.output.expanduser().resolve()
        if args.output is not None
        else default_output_path(input_path, output_dir)
    )
    shifted_tokens = shifted_timestamp_tokens(
        timestamps,
        original_tokens,
        start_row,
        args.start_shift,
        args.end_shift,
    )
    write_shifted_trajectory(
        input_path,
        output_path,
        lines,
        records,
        shifted_tokens,
        args.overwrite,
    )
    if epoch_difference is None:
        print("Epoch 0 selected all trajectory rows, starting at row 0")
    else:
        print(
            f"Matched epoch {args.epoch} to row {start_row}, timestamp "
            f"{timestamps[start_row]} (difference {epoch_difference:+} s)"
        )
    print(
        f"Wrote {len(records)} rows to {output_path}; rows before "
        f"{start_row} unchanged, shift from row {start_row} varies linearly "
        f"from {args.start_shift:+} s to {args.end_shift:+} s"
    )


if __name__ == "__main__":
    main()
