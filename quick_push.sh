#!/bin/bash

# GitHub management script with secure token handling
# Usage: ./quick_push.sh "commit message"

# Check if commit message is provided
if [ -z "$1" ]; then
    echo "Usage: $0 \"commit message\""
    exit 1
fi

# Check if token file exists
TOKEN_FILE="$HOME/.github_token"
if [ ! -f "$TOKEN_FILE" ]; then
    echo "Error: GitHub token file not found at $TOKEN_FILE"
    echo "Run setup_github_token.sh first to configure your token."
    exit 1
fi

# Read token from file
GITHUB_TOKEN=$(cat "$TOKEN_FILE")
if [ -z "$GITHUB_TOKEN" ]; then
    echo "Error: GitHub token is empty"
    exit 1
fi

# Set git credentials using token
git config user.name "wingfeh"
git config user.email "wingfeh@users.noreply.github.com"

# Add all changes
echo "Adding changes..."
git add .

# Check if there are changes to commit
if git diff --staged --quiet; then
    echo "No changes to commit."
    exit 0
fi

# Commit with provided message
echo "Committing changes..."
git commit -m "$1"

# Push to GitHub using token authentication
echo "Pushing to GitHub..."
git push https://${GITHUB_TOKEN}@github.com/wingfeh/Quick.git main

if [ $? -eq 0 ]; then
    echo "✅ Successfully pushed to GitHub!"
else
    echo "❌ Failed to push to GitHub."
    exit 1
fi
