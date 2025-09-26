import os
import sys

# Get current working directory
current_dir = os.getcwd()
print(f"Current directory: {current_dir}")

# List contents of current directory
print(f"\nContents of current directory:")
try:
    items = os.listdir(current_dir)
    for item in items:
        item_path = os.path.join(current_dir, item)
        if os.path.isdir(item_path):
            print(f"📁 {item}/")
        else:
            print(f"📄 {item}")
except Exception as e:
    print(f"Error listing directory: {e}")

# Check specifically for ai_navigator
ai_navigator_path = os.path.join(current_dir, "ai_navigator")
print(f"\nChecking for ai_navigator at: {ai_navigator_path}")
print(f"Exists: {os.path.exists(ai_navigator_path)}")
print(f"Is directory: {os.path.isdir(ai_navigator_path)}")

# If it exists, show its contents
if os.path.exists(ai_navigator_path):
    print(f"\nContents of ai_navigator:")
    try:
        ai_items = os.listdir(ai_navigator_path)
        for item in ai_items:
            print(f"  📄 {item}")
    except Exception as e:
        print(f"  Error: {e}")
