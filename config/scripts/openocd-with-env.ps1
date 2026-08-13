. "$PSScriptRoot\powershell-common.ps1"

$projectRoot = Get-SensWearProjectRoot
Push-Location $projectRoot
try {
	Import-SensWearDotEnv -ProjectRoot $projectRoot
	$openocd = Get-SensWearConfiguredExecutable -VariableName "OPENOCD" `
		-MissingMessage "OPENOCD is not set. Copy .env.example to .env and set OPENOCD."
	if ($null -eq $openocd) {
		exit 1
	}

	$toolArguments = @()
	if (-not [string]::IsNullOrWhiteSpace($env:OPENOCD_SCRIPTS)) {
		$toolArguments += @("-s", $env:OPENOCD_SCRIPTS)
	}
	$toolArguments += $args

	& $openocd @toolArguments
	exit $LASTEXITCODE
} finally {
	Pop-Location
}
