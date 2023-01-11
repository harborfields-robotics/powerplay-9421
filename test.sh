git filter-branch --force --index-filter \
  'git rm --cached --ignore-unmatch java_pid15144.hprof' \
  --prune-empty --tag-name-filter cat -- --all
