# ---------------------------------------------------------------------------
# check_style.awk - Rule 0 and project coding style enforcement
#
# Checks every file passed on the command line for:
#   1. Line length <= 80 characters
#   2. No // comments
#   3. No tab characters or CR (\r) line endings
#   4. No trailing whitespace
#   5. Allman braces: no cuddled '{' or '}' on control-flow lines,
#      including 'else {' and '} else'
#   6. No forbidden keywords (goto, continue)
#   7. No relative #include paths ("../")
#   8. Every file ends in a single newline (no blank line at end of file)
#
# POSIX awk has no word-boundary escape: in mawk and gawk "\b" inside a
# regex is a backspace, so word boundaries are written as explicit
# character classes ((^|[^a-zA-Z0-9_]) and ([^a-zA-Z0-9_]|$)).
# ---------------------------------------------------------------------------

BEGIN {
    errors = 0
    prev_file = ""
    prev_line = ""
    prev_fnr = 0
}

{
    if ((FILENAME != prev_file) && ("" != prev_file))
    {
        check_file_end()
    }

    line = $0
    sub(/\r$/, "", line)

    if ($0 ~ /\r$/) {
        printf("%s:%d: CRLF line ending\n", FILENAME, FNR)
        errors++
    }

    if (length(line) > 80) {
        printf("%s:%d: line length %d > 80\n", FILENAME, FNR, length(line))
        errors++
    }

    if (line ~ /\t/) {
        printf("%s:%d: tab character found\n", FILENAME, FNR)
        errors++
    }

    if (line ~ /[ \t]+$/) {
        printf("%s:%d: trailing whitespace\n", FILENAME, FNR)
        errors++
    }

    if (line ~ /\/\//) {
        printf("%s:%d: forbidden // comment\n", FILENAME, FNR)
        errors++
    }

    if (line !~ /^[ \t]*(\/\*|\*)/) {
        if (line ~ /\)[ \t]*\{/ \
            || line ~ /(^|[^a-zA-Z0-9_])else[ \t]*\{/ \
            || line ~ /\}[ \t]*else([^a-zA-Z0-9_]|$)/) {
            printf("%s:%d: cuddled brace violates Allman style\n", FILENAME, FNR)
            errors++
        }

        if (line ~ /^[ \t]*#[ \t]*include/ && line ~ /\.\.\//) {
            printf("%s:%d: relative #include path (\"../\")\n", FILENAME, FNR)
            errors++
        }

        if (line ~ /(^|[^a-zA-Z0-9_])(goto|continue)([^a-zA-Z0-9_]|$)/) {
            printf("%s:%d: forbidden keyword (goto/continue)\n", FILENAME, FNR)
            errors++
        }
    }

    prev_file = FILENAME
    prev_line = line
    prev_fnr = FNR
}

# A file must end in a single newline: its last line carries code or a
# comment, never only whitespace (checked when the next file starts and
# once more at END, because POSIX awk has no ENDFILE).
function check_file_end()
{
    if (prev_line ~ /^[ \t]*$/) {
        printf("%s:%d: blank line at end of file\n", prev_file, prev_fnr)
        errors++
    }
}

END {
    if ("" != prev_file)
    {
        check_file_end()
    }

    if (errors > 0) {
        printf("Style check FAILED: %d error(s)\n", errors)
        exit 1
    }
}
