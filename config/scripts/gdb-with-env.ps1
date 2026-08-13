. "$PSScriptRoot\powershell-common.ps1"

$projectRoot = Get-SensWearProjectRoot
Push-Location $projectRoot
try {
	Import-SensWearDotEnv -ProjectRoot $projectRoot
	Add-SensWearToolchainToPath
	$gdb = Get-SensWearConfiguredExecutable -VariableName "ZEPHYR_GDB" `
		-MissingMessage "ZEPHYR_GDB is not set. Copy .env.example to .env and set ZEPHYR_GDB."
	if ($null -eq $gdb) {
		exit 1
	}

	# Flashing and verification can take longer than GDB's two-second default.
	& $gdb -iex "set remotetimeout 60" @args
	exit $LASTEXITCODE
} finally {
	Pop-Location
}
