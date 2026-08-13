Set-StrictMode -Version Latest

function Get-SensWearProjectRoot {
	return [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\.."))
}

function Import-SensWearDotEnv {
	param([Parameter(Mandatory = $true)][string]$ProjectRoot)

	$envFile = Join-Path $ProjectRoot ".env"
	if (-not (Test-Path -LiteralPath $envFile -PathType Leaf)) {
		return
	}

	foreach ($rawLine in Get-Content -LiteralPath $envFile) {
		$line = $rawLine.Trim()
		if ($line.Length -eq 0 -or $line.StartsWith("#")) {
			continue
		}

		if ($line.StartsWith("export ")) {
			$line = $line.Substring(7).TrimStart()
		}

		$separator = $line.IndexOf("=")
		if ($separator -le 0) {
			continue
		}

		$name = $line.Substring(0, $separator).Trim()
		if ($name -notmatch "^[A-Za-z_][A-Za-z0-9_]*$") {
			throw "Invalid environment variable name in ${envFile}: $name"
		}

		$value = $line.Substring($separator + 1).Trim()
		if ($value.Length -ge 2) {
			$quotedWithDouble = $value.StartsWith('"') -and $value.EndsWith('"')
			$quotedWithSingle = $value.StartsWith("'") -and $value.EndsWith("'")
			if ($quotedWithDouble -or $quotedWithSingle) {
				$value = $value.Substring(1, $value.Length - 2)
			}
		}

		[Environment]::SetEnvironmentVariable($name, $value, "Process")
	}
}

function Add-SensWearToolchainToPath {
	if ([string]::IsNullOrWhiteSpace($env:NCS_TOOLCHAIN_ROOT)) {
		return
	}

	$toolchainPaths = @(
		(Join-Path $env:NCS_TOOLCHAIN_ROOT "opt\zephyr-sdk\gnu\arm-zephyr-eabi\bin"),
		(Join-Path $env:NCS_TOOLCHAIN_ROOT "opt\bin"),
		(Join-Path $env:NCS_TOOLCHAIN_ROOT "opt\bin\Scripts"),
		(Join-Path $env:NCS_TOOLCHAIN_ROOT "bin")
	)
	$env:PATH = ($toolchainPaths + @($env:PATH)) -join [System.IO.Path]::PathSeparator
}

function Get-SensWearConfiguredExecutable {
	param(
		[Parameter(Mandatory = $true)][string]$VariableName,
		[Parameter(Mandatory = $true)][string]$MissingMessage
	)

	$value = [Environment]::GetEnvironmentVariable($VariableName, "Process")
	if ([string]::IsNullOrWhiteSpace($value)) {
		Write-Error $MissingMessage
		return $null
	}

	if (-not (Test-Path -LiteralPath $value -PathType Leaf)) {
		Write-Error "$VariableName does not point to an executable: $value"
		return $null
	}

	return $value
}

function Find-SensWearSiblingTool {
	param(
		[Parameter(Mandatory = $true)][string]$VariableName,
		[Parameter(Mandatory = $true)][string]$FileName,
		[Parameter(Mandatory = $true)][string]$GdbReplacement
	)

	$value = [Environment]::GetEnvironmentVariable($VariableName, "Process")
	if (-not [string]::IsNullOrWhiteSpace($value)) {
		return $value
	}

	if (-not [string]::IsNullOrWhiteSpace($env:NCS_TOOLCHAIN_ROOT)) {
		$candidate = Join-Path $env:NCS_TOOLCHAIN_ROOT "opt\zephyr-sdk\gnu\arm-zephyr-eabi\bin\$FileName"
		if (Test-Path -LiteralPath $candidate -PathType Leaf) {
			return $candidate
		}
	}

	if (-not [string]::IsNullOrWhiteSpace($env:ZEPHYR_GDB)) {
		return $env:ZEPHYR_GDB -replace "-gdb(\.exe)?$", "-$GdbReplacement`$1"
	}

	return $null
}
