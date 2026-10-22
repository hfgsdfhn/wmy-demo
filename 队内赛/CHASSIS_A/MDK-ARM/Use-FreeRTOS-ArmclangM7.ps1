[CmdletBinding()]
param(
    [switch]$Offline,
    [string]$ProjectFile
)

$ErrorActionPreference = 'Stop'

# The Keil project belongs beside this script; CubeMX sources are one level up.
$projectDirectory = $PSScriptRoot
$sourceRoot = Split-Path -Parent $PSScriptRoot
$configFile = Join-Path $sourceRoot 'Core\Inc\FreeRTOSConfig.h'
$portDirectory = Join-Path $sourceRoot 'Middlewares\Third_Party\FreeRTOS\Source\portable\GCC\ARM_CM7\r0p1'
$portFile = Join-Path $portDirectory 'port.c'
$portMacroFile = Join-Path $portDirectory 'portmacro.h'
$generatedPortDirectory = Join-Path $sourceRoot 'Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS\ARM_CM4F'
$generatedPortFile = Join-Path $generatedPortDirectory 'port.c'
$generatedPortMacroFile = Join-Path $generatedPortDirectory 'portmacro.h'
$kernelTag = 'V10.6.2'
$sourceBase = "https://raw.githubusercontent.com/FreeRTOS/FreeRTOS-Kernel/$kernelTag/portable/GCC/ARM_CM7/r0p1"
$oldIncludePath = '../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F'
$newIncludePath = '../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1'
$newPortPath = '../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c'
$generatedPortPath = '../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c'

if ([string]::IsNullOrWhiteSpace($ProjectFile)) {
    $keilProjects = @(Get-ChildItem -LiteralPath $projectDirectory -File -Filter '*.uvprojx')
    if ($keilProjects.Count -eq 0) {
        throw "No Keil .uvprojx project was found in $projectDirectory. Restore or regenerate the Keil project before running this script."
    }
    if ($keilProjects.Count -gt 1) {
        $choices = ($keilProjects.FullName -join [Environment]::NewLine)
        throw "More than one Keil project was found. Run the PowerShell script with -ProjectFile and select one:`n$choices"
    }
    $projectFile = $keilProjects[0].FullName
} else {
    if (-not [IO.Path]::IsPathRooted($ProjectFile)) {
        $ProjectFile = Join-Path $projectDirectory $ProjectFile
    }
    $projectFile = [IO.Path]::GetFullPath($ProjectFile)
}

if (-not (Test-Path -LiteralPath $projectFile -PathType Leaf)) {
    throw "Keil project was not found: $projectFile"
}
if ((Get-Item -LiteralPath $projectFile).Length -eq 0) {
    throw "Keil project is empty: $projectFile. Restore or regenerate it before running this script."
}
if (-not (Test-Path -LiteralPath $configFile)) {
    throw "FreeRTOSConfig.h was not found: $configFile"
}

New-Item -ItemType Directory -Force -Path $portDirectory | Out-Null

function Get-FreeRTOSPortFile {
    param([string]$Name)

    $destination = Join-Path $portDirectory $Name
    if ($Offline) {
        if (-not (Test-Path -LiteralPath $destination)) {
            throw "Offline mode requires $destination to already exist."
        }
        return
    }

    Write-Host "Downloading FreeRTOS $kernelTag ARM_CM7/r0p1/$Name"
    Invoke-WebRequest -Uri "$sourceBase/$Name" -OutFile $destination
}

Get-FreeRTOSPortFile 'port.c'
Get-FreeRTOSPortFile 'portmacro.h'

# FreeRTOS' GCC port checks __VFP_FP__. Armclang exposes equivalent FPU state
# through __ARM_FP, so accept either macro while still rejecting soft-float builds.
$portContent = Get-Content -LiteralPath $portFile -Raw
$fpuCheck = '#ifndef\s+__VFP_FP__\s*\r?\n\s*#error This port can only be used when the project options are configured to enable hardware floating point support\.\s*\r?\n#endif'
$armclangFpuCheck = @'
#if !defined( __VFP_FP__ ) && ( !defined( __ARM_FP ) || ( __ARM_FP == 0 ) )
    #error This port can only be used when the project options are configured to enable hardware floating point support.
#endif
'@.Trim()
if ($portContent -notmatch '__ARM_FP') {
    $updatedPortContent = [regex]::Replace($portContent, $fpuCheck, $armclangFpuCheck, 1)
    if ($updatedPortContent -eq $portContent) {
        throw 'The downloaded port.c has an unexpected FPU check. Update this script before using the port.'
    }
    $portContent = $updatedPortContent
    Set-Content -LiteralPath $portFile -Value $portContent -NoNewline
}

# CubeMX always emits the RVDS/ARM_CM4F path for this MDK template.  Keep that
# single project entry, but make the generated location contain the M7 port.
Copy-Item -LiteralPath $portFile -Destination $generatedPortFile -Force
Copy-Item -LiteralPath $portMacroFile -Destination $generatedPortMacroFile -Force

$projectContent = Get-Content -LiteralPath $projectFile -Raw
if ([string]::IsNullOrWhiteSpace($projectContent)) {
    throw "Keil project has no readable XML content: $projectFile. Restore or regenerate it before running this script."
}
try {
    [xml]$projectXml = $projectContent
} catch {
    throw "Keil project is not valid XML: $projectFile. Restore or regenerate it before running this script."
}
# CubeMX adds an RVDS Cortex-M4 port to MDK projects.  Do not rewrite its path:
# that leaves two M7 port.c entries.  Remove every legacy entry, then retain one
# GCC Cortex-M7 port entry.
[xml]$projectXml = $projectContent
$portFiles = @($projectXml.SelectNodes('//File') | Where-Object {
    $filePathNode = $_.SelectSingleNode('./FilePath')
    $filePathNode -and $filePathNode.InnerText -match '/portable/(RVDS/ARM_CM4F|GCC/ARM_CM7/r0p1)/port\.c$'
})
$hasGeneratedPort = @($portFiles | Where-Object {
    $_.SelectSingleNode('./FilePath').InnerText -eq $generatedPortPath
}).Count -gt 0

foreach ($file in $portFiles) {
    if ($file.SelectSingleNode('./FilePath').InnerText -ne $generatedPortPath) {
        [void]$file.ParentNode.RemoveChild($file)
    }
}

if (-not $hasGeneratedPort) {
    throw "CubeMX did not generate the expected FreeRTOS port: $generatedPortPath"
}

$projectContent = $projectXml.OuterXml
$projectContent = $projectContent.Replace(";../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1", "")
$projectContent = $projectContent.Replace("../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2;", "../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2;../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F;")
if ($projectContent -notmatch 'FPU3\(DFPU\)') {
    throw 'The Keil target does not enable double-precision hardware FPU. Set Target -> FPU to Double Precision FPU (FPv5-D16), then rerun this script.'
}
Set-Content -LiteralPath $projectFile -Value $projectContent -NoNewline

$configContent = Get-Content -LiteralPath $configFile -Raw
$legacyCompilerTest = 'defined\(__ICCARM__\) \|\| defined\(__CC_ARM\) \|\| defined\(__GNUC__\)'
if ($configContent -match $legacyCompilerTest) {
    $configContent = [regex]::Replace(
        $configContent,
        $legacyCompilerTest,
        'defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__) || defined(__clang__)',
        1)
    Set-Content -LiteralPath $configFile -Value $configContent -NoNewline
}

Write-Host ''
Write-Host 'FreeRTOS Armclang Cortex-M7 port is configured.' -ForegroundColor Green
Write-Host 'In Keil, select Compiler Version 6 and run Project -> Clean Targets, then Build Target.'
Write-Host 'After CubeMX regenerates FreeRTOS files, run this script again.'
