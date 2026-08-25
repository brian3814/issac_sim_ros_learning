@echo off
:: Lesson 5: generate a labelled pallet dataset with Replicator.
::
::   run\dataset.cmd
::   run\dataset.cmd --frames 100
::   run\dataset.cmd --gui          :: watch the randomisation happen

call "%~dp0_common.cmd" "scripts\03_generate_dataset.py" %*
exit /b %ERRORLEVEL%
