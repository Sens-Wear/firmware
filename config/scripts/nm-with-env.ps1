. "$PSScriptRoot\powershell-common.ps1"

$projectRoot = Get-SensWearProjectRoot
Push-Location $projectRoot
try {
	Import-SensWearDotEnv -ProjectRoot $projectRoot
	$nm = Find-SensWearSiblingTool -VariableName "ZEPHYR_NM" `
		-FileName "arm-zephyr-eabi-nm.exe" -GdbReplacement "nm"
	if ([string]::IsNullOrWhiteSpace($nm) -or -not (Test-Path -LiteralPath $nm -PathType Leaf)) {
		Write-Error "ZEPHYR_NM was not found. Set ZEPHYR_GDB or ZEPHYR_NM in .env."
		exit 1
	}

	& $nm @args
	exit $LASTEXITCODE
} finally {
	Pop-Location
}
