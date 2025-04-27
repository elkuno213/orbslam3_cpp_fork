#!/bin/bash
set -e

# Usage: ./format.sh dir1 dir2 "dir with spaces" ...

for dir in "$@"; do
  # Check if the argument is a directory
  if [ -d "$dir" ]; then
    echo "Formatting files in: $dir"
    find "$dir" \
      \( -iname "*.h" -o -iname "*.hpp" -o -iname "*.c" -o -iname "*.cpp" -o -iname "*.cc" \) \
      -exec clang-format-15 -style=file -i {} +
  else
    echo "Warning: '$dir' is not a directory, skipping."
  fi
done

echo "Formatting complete."
