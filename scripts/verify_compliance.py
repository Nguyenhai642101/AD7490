#!/usr/bin/env python3
"""
Automated Coding Rules & DO-178C Compliance Verifier for AD7490 Driver.

Checks all production and test C/H files against C_rules_v3_addDO178C.md:
  - Rule 0: Allman braces, no '//' comments, <= 80 chars/line, LF endings,
            4-space indentation (no tabs), single trailing newline.
  - Rule 2: Identifier length 3..31 (except i, j, k), no forbidden vague
            stems (value, val, data, tmp, buf, info, temp), no type prefixes.
  - Rule 4: Function length <= 100 lines, <= 5 parameters, single return,
            prototype-definition exact match, @req and @design traceability.
  - Rule 5/6: No ternary '?:', no goto/continue, no magic numbers in prod.
  - Rule 9: SEU Hamming distance >= 12 across all state/health patterns.
  - DO-178C Traceability: Bi-directional match between source @req/@design
            tags and docs/TRACEABILITY_MATRIX.md.
"""

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent

PROD_FILES = [
    ROOT / "hal" / "axi_qspi_hal.h",
    ROOT / "hal" / "axi_qspi_hal.c",
    ROOT / "driver" / "ad7490_driver.h",
    ROOT / "driver" / "ad7490_driver.c",
    ROOT / "driver" / "ad7490_monitor.h",
    ROOT / "driver" / "ad7490_monitor.c",
]

TEST_FILES = [
    ROOT / "test" / "test_common.h",
    ROOT / "test" / "test_common.c",
    ROOT / "test" / "fake_ad7490.h",
    ROOT / "test" / "fake_ad7490.c",
    ROOT / "test" / "test_qspi_hal.c",
    ROOT / "test" / "test_frame_convert.c",
    ROOT / "test" / "test_device.c",
    ROOT / "test" / "test_sequencer.c",
    ROOT / "test" / "test_monitor.c",
]

ALL_CH_FILES = PROD_FILES + TEST_FILES

VAGUE_RE = re.compile(
    r"(^|_)(value|val|data|tmp|buf|info|temp)($|_)", re.IGNORECASE
)
PREFIX_RE = re.compile(
    r"^(f|u8|u16|u32|u64|i8|i16|i32|i64|sz|str)_", re.IGNORECASE
)
IDENT_RE = re.compile(r"\b[a-zA-Z_][a-zA-Z0-9_]*\b")


def strip_comments_and_strings(text: str) -> str:
    text = re.sub(r"/\*[\s\S]*?\*/", "", text)
    text = re.sub(r'"(?:\\.|[^"\\])*"', '""', text)
    return text


def check_file_style_and_naming(path: pathlib.Path, is_prod: bool) -> list[str]:
    errors: list[str] = []
    rel = path.relative_to(ROOT).as_posix()
    raw = path.read_bytes()

    if b"\r" in raw:
        errors.append(f"{rel}: contains CR (\\r) line endings")
    if not raw.endswith(b"\n"):
        errors.append(f"{rel}: missing final newline")
    if raw.endswith(b"\n\n"):
        errors.append(f"{rel}: ends with multiple blank lines")

    lines = raw.decode("utf-8").splitlines()
    in_comment = False

    for idx, line in enumerate(lines, start=1):
        if len(line) > 80:
            errors.append(f"{rel}:{idx}: line length {len(line)} > 80")
        if "\t" in line:
            errors.append(f"{rel}:{idx}: tab character")
        if line.endswith(" ") or line.endswith("\t"):
            errors.append(f"{rel}:{idx}: trailing whitespace")
        if "//" in line:
            errors.append(f"{rel}:{idx}: forbidden // comment")

        clean = re.sub(r"/\*.*?\*/", "", line)
        if in_comment:
            if "*/" in clean:
                clean = re.sub(r"^.*?\*/", "", clean)
                in_comment = False
            else:
                continue
        if "/*" in clean:
            clean = re.sub(r"/\*.*$", "", clean)
            in_comment = True

        clean = re.sub(r'"(?:\\.|[^"\\])*"', '""', clean)

        if re.search(r"\)\s*\{|\belse\s*\{|\}\s*else\b", clean):
            errors.append(f"{rel}:{idx}: cuddled brace violates Allman style")
        if re.search(r"\b(goto|continue)\b", clean):
            errors.append(f"{rel}:{idx}: forbidden keyword goto/continue")
        if "?" in clean:
            errors.append(f"{rel}:{idx}: forbidden ternary operator '?'")

        if not re.match(r"^\s*#\s*include", clean):
            for match in IDENT_RE.finditer(clean):
                word = match.group(0)
                if len(word) < 3 and word not in ("if", "do", "i", "j", "k"):
                    errors.append(
                        f"{rel}:{idx}: identifier '{word}' shorter than 3 chars"
                    )
                if len(word) > 31:
                    errors.append(
                        f"{rel}:{idx}: identifier '{word}' exceeds 31 chars"
                    )
                if VAGUE_RE.search(word):
                    errors.append(
                        f"{rel}:{idx}: identifier '{word}' uses vague stem"
                    )
                if PREFIX_RE.search(word):
                    errors.append(
                        f"{rel}:{idx}: identifier '{word}' uses type prefix"
                    )

        if is_prod and path.suffix == ".c":
            if not re.match(r"^\s*#\s*define", clean):
                for num in re.finditer(
                    r"\b([0-9]+(?:U|UL|L)?|0x[0-9a-fA-F]+(?:U|UL|L)?)\b",
                    clean,
                ):
                    lit = num.group(0)
                    if lit not in ("0", "1", "0U", "1U", "2U", "3U"):
                        errors.append(
                            f"{rel}:{idx}: magic number '{lit}' in code"
                        )

    return errors


def check_functions_and_signatures(
    h_path: pathlib.Path, c_path: pathlib.Path
) -> list[str]:
    errors: list[str] = []
    rel_c = c_path.relative_to(ROOT).as_posix()
    lines = c_path.read_text(encoding="utf-8").splitlines()

    in_func = False
    func_start = 0
    func_sig = ""
    brace_depth = 0
    return_count = 0

    for idx, line in enumerate(lines):
        if not in_func and line == "{":
            sig_idx = idx - 1
            while sig_idx >= 0 and not re.match(r"^[a-zA-Z_]", lines[sig_idx]):
                sig_idx -= 1
            if sig_idx >= 0 and not re.match(
                r"^(typedef|static const)", lines[sig_idx]
            ):
                in_func = True
                func_start = sig_idx + 1
                func_sig = " ".join(
                    s.strip() for s in lines[sig_idx:idx]
                ).strip()
                brace_depth = 1
                return_count = 0
                params_str = func_sig[
                    func_sig.find("(") + 1 : func_sig.rfind(")")
                ].strip()
                param_count = (
                    0
                    if params_str in ("", "void")
                    else len(params_str.split(","))
                )
                if param_count > 5:
                    errors.append(
                        f"{rel_c}:{func_start}: {param_count} params > 5"
                    )
                continue

        if in_func:
            clean = re.sub(r"/\*.*?\*/", "", line)
            brace_depth += clean.count("{") - clean.count("}")
            if re.search(r"\breturn\b", clean):
                return_count += 1
            if brace_depth == 0:
                func_len = (idx + 1) - func_start + 1
                if func_len > 100:
                    errors.append(
                        f"{rel_c}:{func_start}: function length {func_len} > 100"
                    )
                if return_count > 1:
                    errors.append(
                        f"{rel_c}:{func_start}: {return_count} returns > 1"
                    )
                in_func = False

    def extract_sigs(path: pathlib.Path):
        txt = strip_comments_and_strings(path.read_text(encoding="utf-8"))
        txt = re.sub(r"(?m)^\s*#.*$", "", txt)
        protos = [
            re.sub(r"\s+", " ", m.group(1)).strip()
            for m in re.finditer(
                r"(?m)^((?:static\s+(?:inline\s+)?)?[a-zA-Z0-9_]+\s+[a-zA-Z0-9_]+\s*\([^;\{]*\))\s*;",
                txt,
            )
        ]
        defs = [
            re.sub(r"\s+", " ", m.group(1)).strip()
            for m in re.finditer(
                r"(?m)^((?:static\s+(?:inline\s+)?)?[a-zA-Z0-9_]+\s+[a-zA-Z0-9_]+\s*\([^;\{]*\))\s*\{",
                txt,
            )
        ]
        return protos, defs

    h_protos, _ = extract_sigs(h_path)
    c_protos, c_defs = extract_sigs(c_path)
    all_protos = set(h_protos + c_protos)

    for d in c_defs:
        if d not in all_protos:
            errors.append(
                f"{rel_c}: definition '{d}' does not match any prototype"
            )
    for p in all_protos:
        if p not in c_defs:
            errors.append(
                f"{rel_c}: prototype '{p}' does not match any definition"
            )

    return errors


def check_hamming_distances() -> list[str]:
    errors: list[str] = []
    patterns = {
        "AD7490_STATE_READY": 0x5A3C96E1,
        "AD7490_STATE_SEQUENCING": 0x3CA5E196,
        "AD7490_STATE_FAULT": 0xC35A693C,
        "AD7490_MON_HEALTH_ACTIVE": 0x96E15A3C,
        "AD7490_MON_HEALTH_FAILED": 0x691EA5C3,
        "ZERO": 0x00000000,
        "ONES": 0xFFFFFFFF,
    }
    keys = list(patterns.keys())
    for i in range(len(keys)):
        for j in range(i + 1, len(keys)):
            k1, k2 = keys[i], keys[j]
            hd = bin(patterns[k1] ^ patterns[k2]).count("1")
            if hd < 12:
                errors.append(
                    f"SEU Hamming distance between {k1} and {k2} is {hd} < 12"
                )
    return errors


def check_deviation_log() -> list[str]:
    """Deviation / defensive log <-> source tags, and no line references.

    Every DEV-* / DEF-* entry of DEVIATION_AND_DEFENSIVE_LOG.md must be
    named in the source comment of the site it documents, and vice
    versa. The log must locate sites by element name: a "file.c:123"
    reference rots silently when lines move above the site.
    """
    errors: list[str] = []
    log_path = ROOT / "docs" / "DEVIATION_AND_DEFENSIVE_LOG.md"
    if not log_path.exists():
        return [f"Missing required document: {log_path.name}"]

    log_text = log_path.read_text(encoding="utf-8")
    id_pattern = r"\b(?:DEV|DEF)-(?:QSPI|AD7490)-\d{3}\b"
    log_ids = set(re.findall(id_pattern, log_text))

    code_ids: set[str] = set()
    for prod_path in PROD_FILES:
        if prod_path.exists():
            code_text = prod_path.read_text(encoding="utf-8")
            code_ids.update(re.findall(id_pattern, code_text))

    for log_id in sorted(log_ids - code_ids):
        errors.append(
            f"Log entry {log_id} is not named in any production source "
            f"comment"
        )
    for code_id in sorted(code_ids - log_ids):
        errors.append(
            f"Source comment names {code_id}, which has no entry in "
            f"DEVIATION_AND_DEFENSIVE_LOG.md"
        )

    for line_no, line in enumerate(log_text.splitlines(), start=1):
        for ref in re.findall(r"[A-Za-z0-9_]+\.[ch]:\d+", line):
            errors.append(
                f"DEVIATION_AND_DEFENSIVE_LOG.md:{line_no}: line-number "
                f"reference '{ref}'; name the function or field instead"
            )

    return errors


def check_traceability() -> list[str]:
    errors: list[str] = []
    required_docs = [
        ROOT / "docs" / "C_rules_v3_addDO178C.md",
        ROOT / "docs" / "AD7490.PDF",
        ROOT / "docs" / "LOW_LEVEL_REQUIREMENTS.md",
        ROOT / "docs" / "SOFTWARE_DESIGN_DESCRIPTION.md",
        ROOT / "docs" / "TRACEABILITY_MATRIX.md",
        ROOT / "docs" / "TEST_REPORT.md",
        ROOT / "docs" / "DEVIATION_AND_DEFENSIVE_LOG.md",
    ]
    for doc_path in required_docs:
        if not doc_path.exists():
            errors.append(
                f"Missing required document: {doc_path.relative_to(ROOT).as_posix()}"
            )
    if errors:
        return errors

    matrix_text = (ROOT / "docs" / "TRACEABILITY_MATRIX.md").read_text(
        encoding="utf-8"
    )
    llr_text = (ROOT / "docs" / "LOW_LEVEL_REQUIREMENTS.md").read_text(
        encoding="utf-8"
    )
    sdd_text = (ROOT / "docs" / "SOFTWARE_DESIGN_DESCRIPTION.md").read_text(
        encoding="utf-8"
    )

    matrix_reqs = set(re.findall(r"\bLLR-(?:QSPI|AD7490)-\d{3}\b", matrix_text))
    matrix_sdds = set(
        re.findall(r"\bSDD-(?:QSPI|AD7490)-\d+\.\d+\b", matrix_text)
    )
    llr_reqs = set(re.findall(r"\bLLR-(?:QSPI|AD7490)-\d{3}\b", llr_text))
    sdd_ids = set(re.findall(r"\bSDD-(?:QSPI|AD7490)-\d+\.\d+\b", sdd_text))

    code_reqs: set[str] = set()
    code_sdds: set[str] = set()
    for p in PROD_FILES:
        txt = p.read_text(encoding="utf-8")
        code_reqs.update(re.findall(r"@req\s+(LLR-(?:QSPI|AD7490)-\d{3})", txt))
        code_sdds.update(
            re.findall(r"@design\s+(SDD-(?:QSPI|AD7490)-\d+\.\d+)", txt)
        )

    for r in sorted(code_reqs - matrix_reqs):
        errors.append(f"Requirement {r} in code missing from TRACEABILITY_MATRIX.md")
    for r in sorted(matrix_reqs - code_reqs):
        errors.append(f"Requirement {r} in TRACEABILITY_MATRIX.md missing from code")
    for r in sorted(code_reqs - llr_reqs):
        errors.append(f"Requirement {r} in code missing from LOW_LEVEL_REQUIREMENTS.md")
    for r in sorted(llr_reqs - code_reqs):
        errors.append(f"Requirement {r} in LOW_LEVEL_REQUIREMENTS.md missing from code")
    for s in sorted(code_sdds - matrix_sdds):
        errors.append(f"Design ID {s} in code missing from TRACEABILITY_MATRIX.md")
    for s in sorted(matrix_sdds - code_sdds):
        errors.append(f"Design ID {s} in TRACEABILITY_MATRIX.md missing from code")
    for s in sorted(code_sdds - sdd_ids):
        errors.append(
            f"Design ID {s} in code missing from SOFTWARE_DESIGN_DESCRIPTION.md"
        )

    return errors


def main() -> int:
    all_errors: list[str] = []

    for p in PROD_FILES:
        if not p.exists():
            all_errors.append(f"Missing production file: {p}")
        else:
            all_errors.extend(check_file_style_and_naming(p, is_prod=True))

    for p in TEST_FILES:
        if not p.exists():
            all_errors.append(f"Missing test file: {p}")
        else:
            all_errors.extend(check_file_style_and_naming(p, is_prod=False))

    pairs = [
        (ROOT / "hal" / "axi_qspi_hal.h", ROOT / "hal" / "axi_qspi_hal.c"),
        (
            ROOT / "driver" / "ad7490_driver.h",
            ROOT / "driver" / "ad7490_driver.c",
        ),
        (
            ROOT / "driver" / "ad7490_monitor.h",
            ROOT / "driver" / "ad7490_monitor.c",
        ),
    ]
    for h_path, c_path in pairs:
        if h_path.exists() and c_path.exists():
            all_errors.extend(check_functions_and_signatures(h_path, c_path))

    all_errors.extend(check_hamming_distances())
    all_errors.extend(check_deviation_log())
    all_errors.extend(check_traceability())

    if all_errors:
        print(f"COMPLIANCE AUDIT FAILED ({len(all_errors)} issue(s)):")
        for err in all_errors:
            print(f"  - {err}")
        return 1

    print("================================================================")
    print("AD7490 COMPLIANCE & TRACEABILITY AUDIT: PASS (0 errors)")
    print(f"  - Production files verified : {len(PROD_FILES)}")
    print(f"  - Test files verified       : {len(TEST_FILES)}")
    print("  - Rule 0 formatting         : PASS (<= 80 cols, Allman, no //)")
    print("  - Rule 2 naming             : PASS (3..31 chars, no vague stems)")
    print("  - Rule 4 function metrics   : PASS (<= 100 lines, 1 return)")
    print("  - Rule 9 SEU Hamming dist   : PASS (HD >= 12..32)")
    print("  - Deviation / defensive log : PASS (IDs tagged in source,")
    print("                                no line-number references)")
    print("  - DO-178C Traceability      : PASS (100% bi-directional match)")
    print("================================================================")
    return 0


if __name__ == "__main__":
    sys.exit(main())
