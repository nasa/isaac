
This folder contains informal test cases for `problem_generator.py` and `plan_survey.py`. The
`*_config.yaml` test cases and the `generate.bash` script are precious source, and all other files
were auto-generated from the test cases and committed so they could be compared to the output of
future runs.

To run the informal tests, call `generate.bash` in this folder, which will overwrite the committed
version of the generated files, then run `git diff` to check for differences in the new outputs
versus their committed version.

Typically we don't expect any differences, but future code changes could change the expected output
and that might not be an error. Observed changes require informed manual review of correctness. If
the new output is correct, it should probably be committed.
