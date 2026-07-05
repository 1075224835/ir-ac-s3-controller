param(
  [string]$ListenAddress = "100.107.138.12",
  [int]$ListenPort = 8080,
  [string]$TargetAddress = "192.168.31.57",
  [int]$TargetPort = 80
)

$ErrorActionPreference = "Stop"

$listener = [System.Net.Sockets.TcpListener]::new(
  [System.Net.IPAddress]::Parse($ListenAddress),
  $ListenPort
)

$listener.Start()
Write-Host "IR AC proxy listening on http://$ListenAddress`:$ListenPort/ -> http://$TargetAddress`:$TargetPort/"
Write-Host "Keep this window open while using the controller remotely."

while ($true) {
  $client = $listener.AcceptTcpClient()

  [System.Threading.ThreadPool]::QueueUserWorkItem({
    param($state)

    $client = $state.Client
    $target = $null

    try {
      $target = [System.Net.Sockets.TcpClient]::new()
      $target.Connect($state.TargetAddress, $state.TargetPort)

      $clientStream = $client.GetStream()
      $targetStream = $target.GetStream()

      $clientToTarget = $clientStream.CopyToAsync($targetStream)
      $targetToClient = $targetStream.CopyToAsync($clientStream)

      [System.Threading.Tasks.Task]::WaitAny($clientToTarget, $targetToClient) | Out-Null
    } catch {
      Write-Warning $_.Exception.Message
    } finally {
      if ($target) { $target.Close() }
      $client.Close()
    }
  }, [pscustomobject]@{
    Client = $client
    TargetAddress = $TargetAddress
    TargetPort = $TargetPort
  }) | Out-Null
}
