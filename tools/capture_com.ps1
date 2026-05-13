param(
    [string]$PortName = "COM4",
    [int]$Baud = 921600,
    [int]$Seconds = 25,
    [string]$OutFile = "serial_log.txt"
)

$ErrorActionPreference = "Stop"
$port = New-Object System.IO.Ports.SerialPort $PortName, $Baud, None, 8, One
$port.ReadTimeout = 200
$port.NewLine = "`n"
$port.Encoding = [System.Text.Encoding]::GetEncoding("ISO-8859-1")
$port.Open()

$fs = [System.IO.File]::Create($OutFile)
$buf = New-Object byte[] 8192
$end = (Get-Date).AddSeconds($Seconds)

Write-Host "[capture_com] opened $PortName @ $Baud, writing to $OutFile for $Seconds s"

while ((Get-Date) -lt $end) {
    try {
        $n = $port.BaseStream.Read($buf, 0, $buf.Length)
        if ($n -gt 0) {
            $fs.Write($buf, 0, $n)
            $fs.Flush()
        }
    } catch [System.TimeoutException] {
        # no data, keep polling
    }
}

$fs.Close()
$port.Close()
Write-Host "[capture_com] done"
