param(
    [Parameter(Mandatory)][string]$Port,
    [int]$Seconds = 20,
    [string]$Output = 'build/p2b/imu.csv'
)
$ErrorActionPreference = 'Stop'
$path = [IO.Path]::GetFullPath($Output)
New-Item -ItemType Directory -Force (Split-Path $path -Parent) | Out-Null
$serial = [IO.Ports.SerialPort]::new($Port, 115200, [IO.Ports.Parity]::None, 8, [IO.Ports.StopBits]::One)
$serial.DtrEnable = $true
$serial.ReadBufferSize = 1024 * 1024
$serial.ReadTimeout = 200
$stream = $null
$buffer = [byte[]]::new(65536)
$bytes = 0
try {
    $serial.Open()
    $stream = [IO.File]::Create($path)
    $watch = [Diagnostics.Stopwatch]::StartNew()
    while ($watch.Elapsed.TotalSeconds -lt $Seconds) {
        try { $count = $serial.Read($buffer, 0, $buffer.Length) }
        catch [TimeoutException] { continue }
        $stream.Write($buffer, 0, $count)
        $bytes += $count
    }
} finally {
    if ($stream) { $stream.Dispose() }
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
Write-Output "Saved $bytes bytes to $path; $Port released."
