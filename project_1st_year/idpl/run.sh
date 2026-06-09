#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

if [ ! -d "frontend/node_modules" ]; then
  echo "Installing frontend dependencies..."
  cd frontend
  npm install
  cd ..
fi

if [ ! -d "frontend/dist" ]; then
  echo "Building React frontend..."
  cd frontend
  npm run build
  cd ..
fi

echo "Installing Python dependencies..."
pip install -r requirements.txt

echo "Starting VoxBot control server..."
python main.py
