Add-Type -AssemblyName System.Runtime.WindowsRuntime
$profile = [Windows.Networking.Connectivity.NetworkInformation,Windows.Networking.Connectivity,ContentType=WindowsRuntime]::GetInternetConnectionProfile()
$mgr = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager,Windows.Networking.NetworkOperators,ContentType=WindowsRuntime]::CreateFromConnectionProfile($profile)
$clients = $mgr.GetTetheringClients()
foreach ($c in $clients) {
    Write-Host "CLIENT: Mac=$($c.MacAddress) Hostname=$($c.HostNames[0]) IP=$($c.HostNames[0].CanonicalName)"
}
