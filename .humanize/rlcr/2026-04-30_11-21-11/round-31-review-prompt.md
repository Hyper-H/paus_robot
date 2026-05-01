# Code Review Phase - Round 31

This file documents the code review invocation for audit purposes.
Note: `codex review` does not accept prompt input; it performs automated code review based on git diff.

## Review Configuration

- **Base Branch**: main
- **Review Round**: 31
- **Timestamp**: 2026-04-30T20:10:43Z

## What This Phase Does

1. Runs `codex review --base main` to perform automated code review
2. Scans output for `[P0-9]` severity markers indicating issues
3. If issues found: Returns fix prompt to Claude for remediation
4. If no issues: Transitions to Finalize Phase

## Expected Output Format

Codex review outputs issues in this format:
```
- [P0] Critical issue description - /path/to/file.py:line-range
  Detailed explanation of the issue.

- [P1] High priority issue - /path/to/file.py:line-range
  Detailed explanation.
```

## Files Generated

- `round-31-review-prompt.md` - This audit file
- `round-31-review-result.md` - Review output (in loop directory)
- `round-31-codex-review.cmd` - Command invocation (in cache)
- `round-31-codex-review.out` - Stdout capture (in cache)
- `round-31-codex-review.log` - Stderr capture (in cache)
