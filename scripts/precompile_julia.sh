#!/bin/bash
set -e # break when error occur

julia --project=. -e '
using Pkg
Pkg.instantiate()
Pkg.precompile()
'
