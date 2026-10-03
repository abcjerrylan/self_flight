param(
    [Parameter(Mandatory)][string]$CubeMXPath,
    [string]$FirmwareRoot,
    [string]$JavaPath = (Join-Path (Split-Path $CubeMXPath -Parent) 'jre/bin/java.exe')
)
$ErrorActionPreference = 'Stop'
$repository = Split-Path $PSScriptRoot -Parent
$board = Join-Path $repository 'boards/micoair743v2_aio35'
$output = Join-Path $board 'generated'
$ioc = Join-Path $output 'micoair743v2_aio35/micoair743v2_aio35.ioc'
$work = Join-Path $repository 'build/p2a'
New-Item -ItemType Directory -Force $work | Out-Null
$script = @('config load ' + $ioc.Replace('\', '/'))
if ($FirmwareRoot) { $script += 'project setCustomFWPath ' + $FirmwareRoot.Replace('\', '/') }
$script += @('project toolchain STM32CubeIDE', ('project path ' + $output.Replace('\', '/')), 'config save', 'project generate', 'exit')
$scriptPath = Join-Path $work 'generate-board.txt'
$log = Join-Path $work 'generate-board.log'
[IO.File]::WriteAllLines($scriptPath, $script)
& $JavaPath -jar $CubeMXPath -q $scriptPath *> $log
if ($LASTEXITCODE -ne 0 -or (Select-String -Path $log -Pattern '^KO$|Exception|generation failed' -Quiet)) {
    throw "CubeMX generation failed; see $log"
}
& (Join-Path $PSScriptRoot 'normalize-board.ps1') -Repository $repository
Write-Output "CubeMX generation log: $log"
