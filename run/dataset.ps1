# Lesson 5: generate a labelled pallet dataset with Replicator.
#
#   .\run\dataset.ps1
#   .\run\dataset.ps1 --frames 100
#   .\run\dataset.ps1 --gui          # watch the randomisation happen

. (Join-Path $PSScriptRoot '_common.ps1')
Invoke-IsaacPython -Script 'scripts\03_generate_dataset.py' -Arguments $args
exit $LASTEXITCODE
