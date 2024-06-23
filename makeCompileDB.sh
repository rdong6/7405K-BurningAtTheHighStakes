#!/bin/zsh
sed -e '1s/^/[\'$'\n''/' -e '$s/,$/\'$'\n'']/' ./bin/**/*.o.json > compile_commands.json