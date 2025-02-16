cppcheck src -isrc/fmt --clang=/opt/homebrew/opt/llvm@19/bin/clang-19

cppcheck --check-level=exhaustive --cppcheck-build-dir=.cppcheck_cache --project=compile_commands.json -isrc/fmt --enable=warning,performance,portability

cppcheck --check-level=exhaustive -isrc/fmt --enable=all src/main.cpp

cppcheck --check-level=exhaustive --project=compile_commands.json -isrc/fmt --enable=warning,performance,portability