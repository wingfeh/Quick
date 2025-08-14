#!/bin/bash

# GitHub Token Setup Script
# This script helps you securely configure your GitHub Personal Access Token

TOKEN_FILE="$HOME/.github_token"

echo "🔐 GitHub Token Setup"
echo "===================="
echo
echo "This script will help you configure your GitHub Personal Access Token."
echo "The token will be stored securely in: $TOKEN_FILE"
echo

# Check if token file already exists
if [ -f "$TOKEN_FILE" ]; then
    echo "⚠️  Token file already exists!"
    echo "Current token: $(head -c 10 "$TOKEN_FILE")..."
    echo
    read -p "Do you want to replace it? (y/N): " replace
    if [[ ! "$replace" =~ ^[Yy]$ ]]; then
        echo "Setup cancelled."
        exit 0
    fi
fi

echo
echo "📝 Please enter your GitHub Personal Access Token:"
echo "   (You can create one at: https://github.com/settings/tokens)"
echo "   Required permissions: repo (Full control of private repositories)"
echo
read -s -p "Token: " token
echo

if [ -z "$token" ]; then
    echo "❌ No token provided. Setup cancelled."
    exit 1
fi

# Save token to file with secure permissions
echo "$token" > "$TOKEN_FILE"
chmod 600 "$TOKEN_FILE"

echo
echo "✅ Token saved successfully!"
echo "   File: $TOKEN_FILE"
echo "   Permissions: $(ls -l "$TOKEN_FILE" | cut -d' ' -f1)"
echo
echo "🚀 You can now use quick_push.sh to push your code to GitHub!"
