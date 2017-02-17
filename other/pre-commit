#!/bin/bash

echo "Formating *.c *.h *.cpp *.hpp files using WebKit style."
find -name *.hpp -o -name *.cpp -o -name *.h -o -name *.c | xargs clang-format-3.8 -i -style="WebKit"
