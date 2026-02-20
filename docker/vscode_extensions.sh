#!/bin/bash

echo "Checking VS Code Extensions..."

EXTENSIONS_INSTALLED=$(code --list-extensions)

for ext in \
    "ms-python.python" \
    "ms-vscode.cpptools" \
    "ms-vscode.cmake-tools" \
    "GitHub.copilot" \
    "ms-python.debugpy" \
    "ms-python.pylance"; do

    if echo "$EXTENSIONS_INSTALLED" | grep -q "$ext"; then
        echo "✅ $ext already installed. Skipping."
    else
        echo "🔄 Installing $ext..."
        code --install-extension "$ext" || echo "❌ Failed to install $ext"
    fi
done

echo "✔️ VS Code Extensions Setup Complete!"

