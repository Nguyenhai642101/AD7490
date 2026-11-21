<#
.SYNOPSIS
    Automated Coding Rules & DO-178C Compliance Verifier for AD7490 Driver.
.DESCRIPTION
    Verifies all production (hal/, driver/) and test (test/) files against
    C_rules_v3_addDO178C.md and docs/TRACEABILITY_MATRIX.md without requiring
    external tools.
#>

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot

$ProdFiles = @(
    "hal\axi_qspi_hal.h",
    "hal\axi_qspi_hal.c",
    "driver\ad7490_driver.h",
    "driver\ad7490_driver.c",
    "driver\ad7490_monitor.h",
    "driver\ad7490_monitor.c"
)

$ExampleFiles = @(
    "examples\ad7490_scan16_raw.c"
)

$TestFiles = @(
    "test\test_common.h",
    "test\test_common.c",
    "test\fake_ad7490.h",
    "test\fake_ad7490.c",
    "test\test_qspi_hal.c",
    "test\test_frame_convert.c",
    "test\test_device.c",
    "test\test_sequencer.c",
    "test\test_monitor.c"
)

$Errors = New-Object System.Collections.Generic.List[string]

foreach ($rel in ($ProdFiles + $ExampleFiles + $TestFiles)) {
    $isProd = ($rel -in $ProdFiles) -or ($rel -in $ExampleFiles)
    $full = Join-Path $Root $rel
    if (-not (Test-Path $full)) {
        $Errors.Add("Missing file: $rel")
        continue
    }
    $content = [System.IO.File]::ReadAllText($full)
    if ($content.Contains("`r")) { $Errors.Add("$rel : contains CR (\r) line endings") }
    if (-not $content.EndsWith("`n")) { $Errors.Add("$rel : missing final newline") }
    if ($content.EndsWith("`n`n")) { $Errors.Add("$rel : ends with multiple blank lines") }

    $lines = [System.IO.File]::ReadAllLines($full)
    $inComment = $false
    for ($i = 0; $i -lt $lines.Count; $i++) {
        $ln = $i + 1
        $line = $lines[$i]
        if ($line.Length -gt 80) { $Errors.Add("$rel`:$ln : line length $($line.Length) > 80") }
        if ($line -match "`t") { $Errors.Add("$rel`:$ln : tab character") }
        if ($line -match '\s+$') { $Errors.Add("$rel`:$ln : trailing whitespace") }
        if ($line -match '//') { $Errors.Add("$rel`:$ln : forbidden // comment") }

        $clean = $line -replace '/\*.*?\*/', ''
        if ($inComment) {
            if ($clean -match '\*/') {
                $clean = $clean -replace '^.*?\*/', ''
                $inComment = $false
            } else { continue }
        }
        if ($clean -match '/\*') {
            $clean = $clean -replace '/\*.*$', ''
            $inComment = $true
        }
        $clean = $clean -replace '"(?:\\.|[^"\\])*"', '""'

        if ($clean -match '\)\s*\{' -or $clean -match '\belse\s*\{' -or $clean -match '\}\s*else\b') {
            $Errors.Add("$rel`:$ln : cuddled brace violates Allman style")
        }
        if ($clean -match '\b(goto|continue)\b') {
            $Errors.Add("$rel`:$ln : forbidden keyword goto/continue")
        }
        if ($clean -match '\?') {
            $Errors.Add("$rel`:$ln : forbidden ternary operator '?'")
        }

        if ($clean -notmatch '^\s*#\s*include') {
            $words = [regex]::Matches($clean, '\b[a-zA-Z_][a-zA-Z0-9_]*\b')
            foreach ($m in $words) {
                $w = $m.Value
                if ($w.Length -lt 3 -and $w -notin @('if', 'do', 'i', 'j', 'k')) {
                    $Errors.Add("$rel`:$ln : identifier '$w' shorter than 3 chars")
                }
                if ($w.Length -gt 31) {
                    $Errors.Add("$rel`:$ln : identifier '$w' exceeds 31 chars")
                }
                if ($w.ToLower() -match '(^|_)(value|val|data|tmp|buf|info|temp)($|_)') {
                    $Errors.Add("$rel`:$ln : identifier '$w' uses vague stem")
                }
                if ($w.ToLower() -match '^(f|u8|u16|u32|u64|i8|i16|i32|i64|sz|str)_') {
                    $Errors.Add("$rel`:$ln : identifier '$w' uses type prefix")
                }
            }
        }

        $isDefine = ($clean -match '^\s*#\s*define')
        $isEnumConst = ($clean -match '^\s*[A-Z][A-Z0-9_]*\s*=\s*(?:0x[0-9a-fA-F]+|[0-9]+)(?:U|UL|L)?\s*,?\s*$')
        if ($isProd -and $rel.EndsWith(".c") -and (-not $isDefine) -and (-not $isEnumConst)) {
            $nums = [regex]::Matches($clean, '\b([0-9]+(?:U|UL|L)?|0x[0-9a-fA-F]+(?:U|UL|L)?)\b')
            foreach ($nm in $nums) {
                $lit = $nm.Value
                if ($lit -notin @('0', '1', '0U', '1U')) {
                    $Errors.Add("$rel`:$ln : magic number '$lit' in code")
                }
            }
        }
    }
}

# Rule 4: Function length <= 100, <= 5 params, single return, prototype-definition match
function Get-Signatures([string]$FullPath) {
    $txt = [System.IO.File]::ReadAllText($FullPath)
    $txt = [regex]::Replace($txt, '/\*[\s\S]*?\*/', '')
    $txt = [regex]::Replace($txt, '"(?:\\.|[^"\\])*"', '""')
    $txt = [regex]::Replace($txt, '(?m)^\s*#.*$', '')
    $protos = @([regex]::Matches($txt, '(?m)^((?:static\s+(?:inline\s+)?)?[a-zA-Z0-9_]+\s+[a-zA-Z0-9_]+\s*\([^;\{]*\))\s*;') |
        ForEach-Object { ($_.Groups[1].Value -replace '\s+', ' ').Trim() })
    $defs = @([regex]::Matches($txt, '(?m)^((?:static\s+(?:inline\s+)?)?[a-zA-Z0-9_]+\s+[a-zA-Z0-9_]+\s*\([^;\{]*\))\s*\{') |
        ForEach-Object { ($_.Groups[1].Value -replace '\s+', ' ').Trim() })
    return @{ Protos = $protos; Defs = $defs }
}

$Pairs = @(
    @("hal\axi_qspi_hal.h", "hal\axi_qspi_hal.c"),
    @("driver\ad7490_driver.h", "driver\ad7490_driver.c"),
    @("driver\ad7490_monitor.h", "driver\ad7490_monitor.c"),
    @("examples\ad7490_scan16_raw.c", "examples\ad7490_scan16_raw.c")
)

foreach ($pair in $Pairs) {
    $hRel = $pair[0]
    $cRel = $pair[1]
    $hFull = Join-Path $Root $hRel
    $cFull = Join-Path $Root $cRel
    if (-not ((Test-Path $hFull) -and (Test-Path $cFull))) { continue }

    $cLines = [System.IO.File]::ReadAllLines($cFull)
    $inFunc = $false
    $funcStart = 0
    $braceDepth = 0
    $returnCount = 0

    for ($idx = 0; $idx -lt $cLines.Count; $idx++) {
        $line = $cLines[$idx]
        if ((-not $inFunc) -and ($line -eq "{")) {
            $sigIdx = $idx - 1
            while (($sigIdx -ge 0) -and ($cLines[$sigIdx] -notmatch '^[a-zA-Z_]')) {
                $sigIdx--
            }
            if (($sigIdx -ge 0) -and ($cLines[$sigIdx] -notmatch '^(typedef|static const)')) {
                $inFunc = $true
                $funcStart = $sigIdx + 1
                $funcSig = (($cLines[$sigIdx..($idx - 1)] | ForEach-Object { $_.Trim() }) -join " ").Trim()
                $braceDepth = 1
                $returnCount = 0
                $openParen = $funcSig.IndexOf("(")
                $closeParen = $funcSig.LastIndexOf(")")
                $paramsStr = $funcSig.Substring($openParen + 1, $closeParen - $openParen - 1).Trim()
                $paramCount = if ($paramsStr -in @("", "void")) { 0 } else { $paramsStr.Split(",").Count }
                if ($paramCount -gt 5) {
                    $Errors.Add("$cRel`:$funcStart : $paramCount params > 5")
                }
                continue
            }
        }
        if ($inFunc) {
            $clean = $line -replace '/\*.*?\*/', ''
            $opens = ([regex]::Matches($clean, '\{')).Count
            $closes = ([regex]::Matches($clean, '\}')).Count
            $braceDepth += ($opens - $closes)
            if ($clean -match '\breturn\b') {
                $returnCount++
            }
            if ($braceDepth -eq 0) {
                $funcLen = ($idx + 1) - $funcStart + 1
                if ($funcLen -gt 100) {
                    $Errors.Add("$cRel`:$funcStart : function length $funcLen > 100")
                }
                if ($returnCount -gt 1) {
                    $Errors.Add("$cRel`:$funcStart : $returnCount returns > 1")
                }
                $inFunc = $false
            }
        }
    }

    $hSigs = Get-Signatures $hFull
    $cSigs = Get-Signatures $cFull
    $allProtos = @($hSigs.Protos + $cSigs.Protos | Select-Object -Unique)
    foreach ($d in $cSigs.Defs) {
        if ($d -notin $allProtos) {
            $Errors.Add("$cRel : definition '$d' does not match any prototype")
        }
    }
    foreach ($p in $allProtos) {
        if ($p -notin $cSigs.Defs) {
            $Errors.Add("$cRel : prototype '$p' does not match any definition")
        }
    }
}

# Rule 9: SEU Hamming distance >= 12 parsed from headers
$ExpectedPatterns = @(
    "AD7490_STATE_READY",
    "AD7490_STATE_SEQUENCING",
    "AD7490_STATE_FAULT",
    "AD7490_MON_HEALTH_ACTIVE",
    "AD7490_MON_HEALTH_FAILED"
)
$Patterns = @{}
foreach ($hRel in @("driver\ad7490_driver.h", "driver\ad7490_monitor.h")) {
    $hFull = Join-Path $Root $hRel
    if (Test-Path $hFull) {
        $hTxt = [System.IO.File]::ReadAllText($hFull)
        $defMatches = [regex]::Matches($hTxt, '(?m)^\s*#\s*define\s+(AD7490_(?:STATE|MON_HEALTH)_[A-Z0-9_]+)\s+\(?(0x[0-9a-fA-F]+)U?\)?')
        foreach ($dm in $defMatches) {
            $Patterns[$dm.Groups[1].Value] = [Convert]::ToInt64($dm.Groups[2].Value, 16)
        }
    }
}
foreach ($ep in $ExpectedPatterns) {
    if (-not $Patterns.ContainsKey($ep)) {
        $Errors.Add("Missing SEU state/health macro in headers: $ep")
    }
}
$Patterns["ZERO"] = [int64]0
$Patterns["ONES"] = [Convert]::ToInt64("0xFFFFFFFF", 16)
$pKeys = @($Patterns.Keys)
for ($i = 0; $i -lt $pKeys.Count; $i++) {
    for ($j = $i + 1; $j -lt $pKeys.Count; $j++) {
        $k1 = $pKeys[$i]
        $k2 = $pKeys[$j]
        $xorVal = [int64]($Patterns[$k1] -bxor $Patterns[$k2])
        $binStr = [Convert]::ToString($xorVal, 2)
        $hd = ([regex]::Matches($binStr, '1')).Count
        if ($hd -lt 12) {
            $Errors.Add("SEU Hamming distance between $k1 and $k2 is $hd < 12")
        }
    }
}

# Check required docs and bi-directional traceability
$RequiredDocs = @(
    "docs\C_rules_v3_addDO178C.md",
    "docs\AD7490.PDF",
    "docs\LOW_LEVEL_REQUIREMENTS.md",
    "docs\SOFTWARE_DESIGN_DESCRIPTION.md",
    "docs\TRACEABILITY_MATRIX.md",
    "docs\TEST_REPORT.md",
    "docs\DEVIATION_AND_DEFENSIVE_LOG.md"
)
foreach ($doc in $RequiredDocs) {
    if (-not (Test-Path (Join-Path $Root $doc))) {
        $Errors.Add("Missing required document: $doc")
    }
}

$matrixPath = Join-Path $Root "docs\TRACEABILITY_MATRIX.md"
$llrPath    = Join-Path $Root "docs\LOW_LEVEL_REQUIREMENTS.md"
$sddPath    = Join-Path $Root "docs\SOFTWARE_DESIGN_DESCRIPTION.md"
if ((Test-Path $matrixPath) -and (Test-Path $llrPath) -and (Test-Path $sddPath)) {
    $matrixText = [System.IO.File]::ReadAllText($matrixPath)
    $llrText    = [System.IO.File]::ReadAllText($llrPath)
    $sddText    = [System.IO.File]::ReadAllText($sddPath)
    $matrixReqs = @([regex]::Matches($matrixText, '\bLLR-(?:QSPI|AD7490)-\d{3}\b') | ForEach-Object { $_.Value } | Select-Object -Unique)
    $matrixSdds = @([regex]::Matches($matrixText, '\bSDD-(?:QSPI|AD7490)-\d+\.\d+\b') | ForEach-Object { $_.Value } | Select-Object -Unique)
    $llrReqs    = @([regex]::Matches($llrText, '\bLLR-(?:QSPI|AD7490)-\d{3}\b') | ForEach-Object { $_.Value } | Select-Object -Unique)
    $sddIds     = @([regex]::Matches($sddText, '\bSDD-(?:QSPI|AD7490)-\d+\.\d+\b') | ForEach-Object { $_.Value } | Select-Object -Unique)

    $codeReqs = @()
    $codeSdds = @()
    foreach ($rel in $ProdFiles) {
        $txt = [System.IO.File]::ReadAllText((Join-Path $Root $rel))
        $codeReqs += @([regex]::Matches($txt, '@req\s+(LLR-(?:QSPI|AD7490)-\d{3})') | ForEach-Object { $_.Groups[1].Value })
        $codeSdds += @([regex]::Matches($txt, '@design\s+(SDD-(?:QSPI|AD7490)-\d+\.\d+)') | ForEach-Object { $_.Groups[1].Value })
    }
    $codeReqs = @($codeReqs | Select-Object -Unique)
    $codeSdds = @($codeSdds | Select-Object -Unique)

    foreach ($r in $codeReqs) {
        if ($r -notin $matrixReqs) { $Errors.Add("Requirement $r in code missing from TRACEABILITY_MATRIX.md") }
        if ($r -notin $llrReqs) { $Errors.Add("Requirement $r in code missing from LOW_LEVEL_REQUIREMENTS.md") }
    }
    foreach ($r in $matrixReqs) {
        if ($r -notin $codeReqs) { $Errors.Add("Requirement $r in TRACEABILITY_MATRIX.md missing from code") }
    }
    foreach ($r in $llrReqs) {
        if ($r -notin $codeReqs) { $Errors.Add("Requirement $r in LOW_LEVEL_REQUIREMENTS.md missing from code") }
    }
    foreach ($s in $codeSdds) {
        if ($s -notin $matrixSdds) { $Errors.Add("Design ID $s in code missing from TRACEABILITY_MATRIX.md") }
        if ($s -notin $sddIds) { $Errors.Add("Design ID $s in code missing from SOFTWARE_DESIGN_DESCRIPTION.md") }
    }
    foreach ($s in $matrixSdds) {
        if ($s -notin $codeSdds) { $Errors.Add("Design ID $s in TRACEABILITY_MATRIX.md missing from code") }
    }
}

# Deviation / defensive log <-> source tags, and no line-number refs
$logPath = Join-Path $Root "docs\DEVIATION_AND_DEFENSIVE_LOG.md"
if (Test-Path $logPath) {
    $logText = [System.IO.File]::ReadAllText($logPath)
    $idPattern = '\b(?:DEV|DEF)-(?:QSPI|AD7490)-\d{3}\b'
    $logIds = @([regex]::Matches($logText, $idPattern) | ForEach-Object { $_.Value } | Select-Object -Unique)

    $codeIds = @()
    foreach ($rel in $ProdFiles) {
        $txt = [System.IO.File]::ReadAllText((Join-Path $Root $rel))
        $codeIds += @([regex]::Matches($txt, $idPattern) | ForEach-Object { $_.Value })
    }
    $codeIds = @($codeIds | Select-Object -Unique)

    foreach ($id in $logIds) {
        if ($id -notin $codeIds) { $Errors.Add("Log entry $id is not named in any production source comment") }
    }
    foreach ($id in $codeIds) {
        if ($id -notin $logIds) { $Errors.Add("Source comment names $id, which has no entry in DEVIATION_AND_DEFENSIVE_LOG.md") }
    }

    $logLines = [System.IO.File]::ReadAllLines($logPath)
    for ($i = 0; $i -lt $logLines.Count; $i++) {
        foreach ($m in [regex]::Matches($logLines[$i], '[A-Za-z0-9_]+\.[ch]:\d+')) {
            $Errors.Add("DEVIATION_AND_DEFENSIVE_LOG.md:$($i + 1): line-number reference '$($m.Value)'; name the function or field instead")
        }
    }
}

if ($Errors.Count -gt 0) {
    Write-Output "COMPLIANCE AUDIT FAILED ($($Errors.Count) issue(s)):"
    foreach ($e in $Errors) { Write-Output "  - $e" }
    exit 1
}

Write-Output "================================================================"
Write-Output "AD7490 COMPLIANCE & TRACEABILITY AUDIT: PASS (0 errors)"
Write-Output "  - Production files verified : $($ProdFiles.Count)"
Write-Output "  - Example files verified    : $($ExampleFiles.Count)"
Write-Output "  - Test files verified       : $($TestFiles.Count)"
Write-Output "  - Rule 0 formatting         : PASS (<= 80 cols, Allman, no //)"
Write-Output "  - Rule 2 naming             : PASS (3..31 chars, no vague stems)"
Write-Output "  - Rule 4 function metrics   : PASS (<= 100 lines, 1 return)"
Write-Output "  - Rule 9 SEU Hamming dist   : PASS (HD >= 12..32)"
Write-Output "  - Deviation / defensive log : PASS (IDs tagged in source,"
Write-Output "                                no line-number references)"
Write-Output "  - DO-178C Traceability      : PASS (100% bi-directional match)"
Write-Output "================================================================"
exit 0
