#!/bin/bash

echo "Checking VS Code Extensions..."

EXTENSIONS_INSTALLED=$(code --list-extensions)

# GitHub.copilot is omitted: Copilot Chat is built into current VS Code and the
# marketplace copilot package tries to downgrade it, which fails.
for ext in \
    "ms-python.python" \
    "ms-vscode.cpptools" \
    "ms-vscode.cmake-tools" \
    "ms-python.debugpy" \
    "ms-python.vscode-pylance"; do

    # Whole-line, fixed-string, case-insensitive match (avoids prefix/regex false hits)
    if echo "$EXTENSIONS_INSTALLED" | grep -qixF "$ext"; then
        echo "[skip] $ext already installed"
    else
        echo "[install] $ext"
        code --install-extension "$ext" || echo "[FAIL] $ext"
    fi
done

echo "VS Code Extensions Setup Complete"
