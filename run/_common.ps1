# Shared helpers for the launcher scripts.
#
# Deliberately, nothing here sets a single environment variable. The Python
# side reads .env and applies what it needs inside its own process, so running
# these scripts -- even by dot-sourcing them into your own shell -- cannot
# change your environment. There is no cleanup step because there is nothing
# to clean up.

function Get-ProjectRoot {
    return (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
}

function Read-DotEnv {
    param([string]$Path)
    $map = @{}
    if (-not (Test-Path $Path)) { return $map }
    foreach ($line in Get-Content $Path) {
        $t = $line.Trim()
        if ($t -eq '' -or $t.StartsWith('#')) { continue }
        $i = $t.IndexOf('=')
        if ($i -lt 1) { continue }
        $k = $t.Substring(0, $i).Trim()
        $v = $t.Substring($i + 1)
        $hash = $v.IndexOf(' #')
        if ($hash -ge 0) { $v = $v.Substring(0, $hash) }
        $map[$k] = $v.Trim().Trim('"').Trim("'")
    }
    return $map
}

function Get-IsaacPython {
    $root = Get-ProjectRoot
    $envFile = Join-Path $root '.env'
    if (-not (Test-Path $envFile)) {
        Write-Host "No .env found." -ForegroundColor Yellow
        Write-Host "  Copy .env.example to .env and edit it:" -ForegroundColor Yellow
        Write-Host "    Copy-Item .env.example .env" -ForegroundColor Yellow
        throw ".env missing"
    }
    $cfg = Read-DotEnv $envFile
    $isaac = $cfg['ISAACSIM_ROOT']
    if ([string]::IsNullOrWhiteSpace($isaac)) { $isaac = 'C:\isaacsim' }
    $py = Join-Path $isaac 'python.bat'
    if (-not (Test-Path $py)) {
        throw "Isaac Sim python.bat not found at $py -- fix ISAACSIM_ROOT in .env"
    }
    return $py
}

function Invoke-IsaacPython {
    param(
        [Parameter(Mandatory = $true)][string]$Script,
        [string[]]$Arguments = @()
    )
    $py = Get-IsaacPython
    $root = Get-ProjectRoot
    Push-Location $root
    try {
        # -u keeps Python's output interleaved with Kit's native logging
        # instead of block-buffered (and lost if the process dies).
        #
        # Note this function deliberately returns nothing. A PowerShell
        # function returns *everything* it emits, so a `return $code` here
        # would bundle the simulator's entire console output into the return
        # value -- and the caller's `exit (...)` would swallow it. Callers read
        # $LASTEXITCODE instead, which the native call above sets globally.
        & $py '-u' $Script @Arguments
    }
    finally {
        Pop-Location
    }
}
