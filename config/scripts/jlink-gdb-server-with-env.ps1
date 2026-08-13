. "$PSScriptRoot\powershell-common.ps1"

$projectRoot = Get-SensWearProjectRoot
Push-Location $projectRoot
try {
	Import-SensWearDotEnv -ProjectRoot $projectRoot
	$jlink = $env:JLINK_GDB_SERVER
	if ([string]::IsNullOrWhiteSpace($jlink)) {
		foreach ($candidate in @("JLinkGDBServerCLExe", "JLinkGDBServerCL", "JLinkGDBServer")) {
			$command = Get-Command $candidate -CommandType Application -ErrorAction SilentlyContinue |
				Select-Object -First 1
			if ($null -ne $command) {
				$jlink = $command.Source
				break
			}
		}
	}

	if ([string]::IsNullOrWhiteSpace($jlink)) {
		Write-Error "J-Link GDB server was not found. Set JLINK_GDB_SERVER in .env."
		exit 1
	}
	if (-not (Test-Path -LiteralPath $jlink -PathType Leaf)) {
		Write-Error "JLINK_GDB_SERVER does not point to an executable: $jlink"
		exit 1
	}

	& $jlink @args
	exit $LASTEXITCODE
} finally {
	Pop-Location
}
