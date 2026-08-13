. "$PSScriptRoot\powershell-common.ps1"

$projectRoot = Get-SensWearProjectRoot
Push-Location $projectRoot
try {
	Import-SensWearDotEnv -ProjectRoot $projectRoot
	$objdump = Find-SensWearSiblingTool -VariableName "ZEPHYR_OBJDUMP" `
		-FileName "arm-zephyr-eabi-objdump.exe" -GdbReplacement "objdump"
	if ([string]::IsNullOrWhiteSpace($objdump) -or -not (Test-Path -LiteralPath $objdump -PathType Leaf)) {
		Write-Error "ZEPHYR_OBJDUMP was not found. Set ZEPHYR_GDB or ZEPHYR_OBJDUMP in .env."
		exit 1
	}

	& $objdump @args
	exit $LASTEXITCODE
} finally {
	Pop-Location
}
