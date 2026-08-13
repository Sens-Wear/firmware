. "$PSScriptRoot\powershell-common.ps1"

$projectRoot = Get-SensWearProjectRoot
Push-Location $projectRoot
try {
	Import-SensWearDotEnv -ProjectRoot $projectRoot
	Add-SensWearToolchainToPath

	& code --new-window . @args
	exit $LASTEXITCODE
} finally {
	Pop-Location
}
