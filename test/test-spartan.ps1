# HakiOS on the realme GT Neo 3T (spartan): the tester's script for Windows.
#
#   powershell -ExecutionPolicy Bypass -File test-spartan.ps1 1     stage 1: U-Boot alone
#   powershell -ExecutionPolicy Bypass -File test-spartan.ps1 2     stage 2: the loader and the open kernel's hello
#   powershell -ExecutionPolicy Bypass -File test-spartan.ps1 2 -FromLoader
#                                     stage 2 from a loader already running (after installing its Windows driver)
#
# The phone in fastboot mode (power off, then power + volume down), on USB. Everything runs from RAM: nothing is
# flashed, and the phone goes back to Android at the end. If ABL refuses an image and changes the active slot (OPLUS ABL
# does that), the script puts the slot back. Logs go to logs\<time>-stage<N>\.
param([Parameter(Mandatory = $true)][ValidateSet('1', '2')][string]$Stage, [switch]$FromLoader)

$ErrorActionPreference = 'Stop'
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Fastboot = 'fastboot'
if ($env:FASTBOOT) { $Fastboot = $env:FASTBOOT }
$Out = Join-Path $Here ('logs\' + (Get-Date -Format 'HHmmss') + '-stage' + $Stage)
New-Item -ItemType Directory -Force $Out | Out-Null

function FB([string]$Arguments) {
	# fastboot prints to stderr: through cmd, so that Windows PowerShell does not take that for an error
	$text = cmd /c "`"$Fastboot`" $Arguments 2>&1"
	Add-Content -Path (Join-Path $Out 'fastboot.txt') -Value ("> fastboot $Arguments`r`n" + ($text -join "`r`n"))
	return ($text -join "`n")
}
function Getvar([string]$Name) {
	$m = [regex]::Match((FB "getvar $Name"), "(?m)^$([regex]::Escape($Name)):\s*(.*)$")
	if ($m.Success) { return $m.Groups[1].Value.Trim() }
	return ''
}
function WaitFor([string]$Product, [int]$Seconds) {
	for ($i = 0; $i -lt $Seconds; $i += 2) {
		if ((Getvar 'product') -like "*$Product*") { return $true }
		Start-Sleep -Seconds 2
	}
	return $false
}
function Say([string]$Text) { Write-Host $Text; Add-Content -Path (Join-Path $Out 'summary.txt') -Value $Text }

# --- the phone, in ABL's fastboot ---------------------------------------------------------------------------------
if ($FromLoader -and $Stage -ne '2') { Say '-FromLoader is for stage 2 only.'; exit 1 }
if ((FB 'devices') -notmatch 'fastboot') { Say 'No phone in fastboot mode: power off, hold power + volume down, connect USB.'; exit 1 }
$slot = ''
if (-not $FromLoader) {
	$product = Getvar 'product'
	if ($product -ne 'kona') { Say "This phone says product '$product', not 'kona' (Snapdragon 870). Stopping."; exit 1 }
	if ((Getvar 'unlocked') -ne 'yes') { Say 'The bootloader is locked: fastboot boot will not work. Stopping.'; exit 1 }
	$slot = Getvar 'current-slot'
	Say "ABL: product kona, unlocked, active slot '$slot'."
}

function PutSlotBack {
	# OPLUS ABL marks the active slot unbootable and switches when it cannot start an image
	Start-Sleep -Seconds 3
	if ((Getvar 'product') -ne 'kona') { return }
	$now = Getvar 'current-slot'
	$bad = Getvar "slot-unbootable:$slot"
	if ($now -ne $slot -or $bad -eq 'yes') {
		Say "ABL refused the image and changed the slots (now '$now', '$slot' unbootable: $bad): setting '$slot' back."
		FB "set_active $slot" | Out-Null
		Say ("Slot now: " + (Getvar 'current-slot') + ", unbootable:" + (Getvar "slot-unbootable:$slot"))
	}
}

if ($Stage -eq '1') {
	Say 'Stage 1: U-Boot alone.'
	$r = FB ('boot "' + (Join-Path $Here 'stage1-u-boot\hakios-u-boot.img') + '"')
	if ($r -notmatch 'Booting\s+OKAY') { Say "ABL did not start it:`n$r"; PutSlotBack; exit 2 }
	Say 'ABL started it. Waiting for U-Boot on USB (Windows may need the Google USB driver for it)...'
	if (-not (WaitFor 'realme-spartan' 40)) {
		Say 'U-Boot did not answer on USB. Look at the screen, take a photo, then hold power 15 s for Android.'
		exit 3
	}
	Say ('U-Boot answers: ' + (Getvar 'version-bootloader'))
	FB 'oem console' | Set-Content -Path (Join-Path $Out 'u-boot-console.txt')
	Say "U-Boot's log: $Out\u-boot-console.txt"
	Say 'Now: is there text on the screen ending in "HakiOS: waiting for the host"? Take a photo.'
	Read-Host 'Press Enter to send the phone back to Android'
	FB 'reboot' | Out-Null
	Say "Done: the phone goes back to Android. Send the whole folder $Out"
	exit 0
}

# --- stage 2 -------------------------------------------------------------------------------------------------------
Say 'Stage 2: U-Boot, the HakiOS loader, the open kernel and hello.'
if (-not $FromLoader) {
	$r = FB ('boot "' + (Join-Path $Here 'stage2-hello\xnu-usb.img') + '"')
	if ($r -notmatch 'Booting\s+OKAY') { Say "ABL did not start it:`n$r"; PutSlotBack; exit 2 }
}
Say 'Waiting for the HakiOS loader on USB...'
if (-not (WaitFor 'xnuloader' 60)) {
	$dev = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object { $_.InstanceId -like 'USB\VID_18D1&PID_D00D\XNULOADER*' }
	if ($dev -and $dev.Status -ne 'OK') {
		Say 'The loader is up, but Windows has no driver for it ("xnuloader" in Device Manager). Leave the phone as it is and:'
		Say '  Device Manager > xnuloader > Update driver > Browse my computer > Let me pick > Android Device > Android Bootloader Interface'
		Say 'Then run:  powershell -ExecutionPolicy Bypass -File test-spartan.ps1 2 -FromLoader'
	} else {
		Say 'The loader did not answer on USB. Take a photo of the screen, then hold power 15 s for Android.'
	}
	exit 3
}
FB 'oem log' | Out-Null
FB ('get_staged "' + (Join-Path $Out 'loader-before.txt') + '"') | Out-Null
Say ('Loader up: version ' + (Getvar 'version') + '. Its log: loader-before.txt (look for "platform spartan").')

$r = FB ('stage "' + (Join-Path $Here 'stage2-hello\run-payload.bin') + '"')
if ($r -notmatch 'OKAY') { Say "Sending the kernel failed:`n$r"; FB 'reboot' | Out-Null; exit 4 }
FB 'oem run' | Out-Null
Say 'The kernel is running (it hands back after about 8 s)...'
$result = ''
for ($i = 0; $i -lt 120; $i += 2) {
	Start-Sleep -Seconds 2
	$result = Getvar 'result'
	if ($result -and $result -notmatch 'running') { break }
}
if (-not $result -or $result -match 'running') {
	Say 'No answer from the loader: the kernel hung. Photo of the screen, then hold power 15 s for Android.'
	exit 5
}
Say "Result: $result"
FB 'oem log' | Out-Null
FB ('get_staged "' + (Join-Path $Out 'run-log.txt') + '"') | Out-Null
$screen = Getvar 'screen'
FB 'oem screenshot' | Out-Null
FB ('get_staged "' + (Join-Path $Out 'screen.raw') + '"') | Out-Null
Set-Content -Path (Join-Path $Out 'screen.txt') -Value "width height bytes-per-row: $screen (raw 32-bit pixels)"
if ((Get-Content (Join-Path $Out 'run-log.txt') -Raw -ErrorAction SilentlyContinue) -match 'Hello from user space on Darwin') {
	Say 'SUCCESS: "Hello from user space on Darwin!" is in the log.'
} else {
	Say 'No hello in the log: send the folder.'
}
FB 'reboot' | Out-Null
Say "Done: the phone goes back to Android. Send the whole folder $Out"
