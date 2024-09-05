#!/usr/bin/env bash
tests=("test/test1.txt" "test/test2.txt" "test/test3.txt")

for item in ${tests[@]}
do
  echo $item
  python main_bow_v0.py --config $item
done
