# THIS IS A WIP
This file is for contributors to Unmanned Life's Git repo.

It still needs:
  - info on code style conventions
  - cleanup to make it easier to parse: it's pretty dense right now

# Python code style (T. Gareau)
[Set up pre-commit hooks](https://pre-commit.com) to run `black` and `flake8` on all code before you commit. `black` is a Python auto-formatter & `flake8` is a PEP8-linter -- if either of these tools fails your commit, fix the code in question or (if you absolutely need to!) ignore the error for that one line.

We use a shared .pre-commit-config.yaml file located in the root of the UPlatform repo.

The only big deviation from PEP8 is that we use 88-character lines.

For `black`, you'll need python3.6+ -- to install this on Ubuntu 16.04 you'll need to add the deadsnakes ppa:

```
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt-get update
sudo apt-get install python3.6
```

# Git Workflow
We use the [Gitflow Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow) to work with our Git repos.

In the Gitflow Workflow, we use two branches to record the history of the project: `master` and `dev`.

The `master` branch stores only the release history -- new commits are intended to represent a new release of some product -- while the `dev` branch serves as the integration branch for features and bug-fixes.

Rule of thumb:
- Small changes (parameter tuning, etc) in the code can be merge directly to `dev`.
- Bigger features & bug fixes, esp. if multiple people will be working on them, go on their own `feat/` branch.
- Small bug fixes for the master branch go on a `hotfix/` branch.
- When the `dev` branch is being prepped to merge into `master`, create a `release/` branch.
  - The `release/` branch marks the end of features that will be merged into `dev` -- only documentation or small bug fixes should be added here.

Eventually, our commit history will look like this:

![gitflow-png](https://github.com/umdlife/UPlatform/raw/dev/docs/gitflow.png)

A typical workflow for features & bug fixes (the most common changes) would look like:

## Create an issue
Create a labeled (enhancement, bug, rover, copter, etc) issue on Github.

## Create a feature branch
Create a new feature branch using `dev` as the parent branch.

```bash
git checkout dev
git pull dev # pull latest changes if required
git checkout -b feat/example
```

## Make changes to the feature branch
```bash
...
git add <some-file>
git commit # with a good commit message
...
```

## Push feature branch to origin
Before creating a pull request, you'll need to push your changes to the remote repository. You may push the feature branch to remote before this to backup your work or to share this branch with another developer.

```bash
git push -u origin feat/example
```

## Create pull request
Create a pull request on Github to pull your feature branch into develop. 

It is also possible to create a pull request before you're ready to commit everything into develop! You can continue to commit changes to the compared branch before merging.

We'll use the pull requests to discuss changes in the code and in the future we plan to introduce structured code reviews.

## Prepare the branch for merge
Here, do any last cleanup tasks before we officially merge the commits associated with this branch into `dev`.

What we want to achieve here is a clean list of atomic commits. The most common action you'll do here is squashing commits. 

For example, say we had the following commits:

```bash
commit 12345

  Add latch timeout functionality to postal rovers

  Great description -----------------------------
  -----------------------------------------------
  -----------------------------------------------
  here.

commit 12344

  WIP branch, backing up

commit 12343

  WIP branch to share with other dev

commit 12342

  Delete `logic.py` as it is no longer used

  `logic.py` is a terror. It has no place in God's 
  plan and I have burned it out of existence.

commit 12341

  Create ConsecutiveReadingDebouncer

  ConsecutiveReadingDebouncer is a class that only
  changes its value after `n` consecutive updates 
  with the same value. It is used in blah blah...
```

Commit 12341 introduces a new class -- that's a single set of changes we want to record in the commit history.

Commit 12342 deletes an old junk file -- that's another change we want to record.

Commits 12343 and 12344 are WIP commits made while finalizing commit 12345 -- we only want to have a single commit here to record the introduction of the latch timeout functionality.

To "squash" these commits together, we have a couple options. The first method, `git rebase -i <after-this-commit>`, allows you to retain the commit author. You must pass the parent of the oldest commit you'd like to squash as `<after-this-commit>`. A shorthand way to do so is to add the caret (^) operator after the commit SHA of the oldest commit you want to squash:

```bash
git rebase -i 12343^
```

We'll then be presented with this window:

```bash
pick 12343 WIP branch to share with other dev
pick 12344 WIP branch, backing up
pick 12345 Add latch timeout functionality to postal rovers

# Rebase 710f0f8..a5f4a0d onto 710f0f8
#
# Commands:
# p, pick <commit> = use commit
# r, reword <commit> = use commit, but edit the commit message
# e, edit <commit> = use commit, but stop for amending
# s, squash <commit> = use commit, but meld into previous commit
# f, fixup <commit> = like "squash", but discard this commit's log message
# x, exec <command> = run command (the rest of the line) using shell
# b, break = stop here (continue rebase later with 'git rebase --continue')
# d, drop <commit> = remove commit
# l, label <label> = label current HEAD with a name
# t, reset <label> = reset HEAD to a label
# m, merge [-C <commit> | -c <commit>] <label> [# <oneline>]
# .       create a merge commit using the original merge commit's
# .       message (or the oneline, if no original merge commit was
# .       specified). Use -c <commit> to reword the commit message.
#
# These lines can be re-ordered; they are executed from top to bottom.
#
# If you remove a line here THAT COMMIT WILL BE LOST.
#
# However, if you remove everything, the rebase will be aborted.
#
# Note that empty commits are commented out
```

To squash these commits, we'll change the first lines to:

```bash
pick 12343 WIP branch to share with other dev
squash 12344 WIP branch, backing up
squash 12345 Add latch timeout functionality to postal rovers
```

then save and quit. Git will apply those changes and plop us into an editor:

```bash
# This is a combination of 3 commits.
# The first commit's message is:
WIP branch to share with other dev

# This is the 2nd commit message:

WIP branch, backing up

# This is the 3rd commit message:

Add latch timeout functionality to postal rovers

  Great description -----------------------------
  -----------------------------------------------
  -----------------------------------------------
  here.

```

which we'll use to replace with a single, well-written commit message.

## A note about rebase
`rebase` rewrites your Git history. This is fine when you're working on a local branch but can wreak havoc if you try to rewrite publically pushed commits.

The Git documentation recommends:
> Don't push your work until you're happy with it!
> One of the cardinal rules of Git is that, since so much work is local within your clone, you have a great deal of freedom to rewrite your history locally. However, once you push your work, it is a different story entirely, and you should consider pushed work as final unless you have good reason to change it. In short, you should avoid pushing your work until you\u2019re happy with it and ready to share it with the rest of the world.

If you ever find Git telling you that you need to `git push --force` (especially to `dev` or `master`), you probably shouldn't!

## Merge the branch
Finally, merge the branch into `dev`. You can do this using the button associated with the pull-request, but if you're doing it from the command-line, make sure to use:

```bash
git merge --no-ff feat/example
```

The --no-ff flag will ensure that the merge will always create a new commit object, even if the changes could be fast-forwarded. This avoids losing information about the historical existence of a feature branch and groups together commits that introduced a feature, making reverting those changes if required a lot easier.

# Branch Naming
Name your branch according to what type of changes you're making. Currently, we only differentiate between two types of branches:
- bug/my_branch  (for bugs)
- feat/my_branch  (for features)
- hack/my_branch  (for dirty quick-fixes that we'll apply to a single project, but we don't want to add to main branches (dev or master))
- release/X.Y.Z  (for specific releases before merging into master)
- hotfix/my_branch  (for quick fixes that we'll merge directly to master to immediately deploy to customers)

Try to keep your names concise.

# Commit Messages
Quick rules for commit messages:
- A short commit message in the imperative ("Fix bug" rather than "Fixed bug" or "Fixes bug")
  - Try to keep this to 50 characters or less, but this is not a hard requirement
- A short body after the commit message, if necessary, SEPARATED BY A NEWLINE
  - Very important to separate by newline: many Git tools rely on it to slice out a short version of the commit message

Here's a model commit message:
```
Capitalized, short (50 chars or less) summary

More detailed explanatory text, if necessary.  Wrap it to about 72
characters or so.  In some contexts, the first line is treated as the
subject of an email and the rest of the text as the body.  The blank
line separating the summary from the body is critical (unless you omit
the body entirely); tools like rebase can get confused if you run the
two together.

Write your commit message in the imperative: "Fix bug" and not "Fixed bug"
or "Fixes bug."  This convention matches up with commit messages generated
by commands like git merge and git revert.

Further paragraphs come after blank lines.

- Bullet points are okay, too

- Typically a hyphen or asterisk is used for the bullet, followed by a
  single space, with blank lines in between, but conventions vary here

- Use a hanging indent
```

# Confluence Code of conduct (A. Sushil)

## 👋 Welcome to developer contribution guideline! 

This page explains how to get involved and help with the technical development and documentation of our platform.

This is a great opportunity for programmers and non-programmers to contribute to defining standards in our workflow as well the  code base.

## ℹ General information

The things that you will find in this documents are:

1. Coding style (mainly c++)
2. Work flow to make a contribution
3. Checklist before requesting a code review
4. Checklist for reviewers

You can browse through the title on the index for more titles.

## 🤝 Code of conduct

Please follow the guidelines as closely as possible. If you have suggestions please bring them to the table. After a discussion we can include them or not. But once agreed upon it is highly important that you follow them. At Unmanned Life, we work together as a team.



*“Documentation is a love letter that you write to your future self“* 

​                                                                                      \- Code Lover Damian Conway

## 👩‍🏭   General Work Flow

This section describes the steps to follow from getting assigned a ticket on JIRA to the merging a pull request.

1. Get a task assigned to you on JIRA.
2. Write a small description of the task and add the estimated time of completion for the task.
3. Move your ticket from TO DO to IN PROGRESS .
4. Create an issue on GitHub for that ticket in the appropriate repository. You may create multiple issues for a ticket in different repos. Make sure your heading is clear to a non-programmer.  Add enough description about what is the issue and how you intent to solve it.
5. Find a buddy and discuss your design. Try to stick with the plan you and buddy agree upon. If there are adaptation to be done on the design talk to your buddy again.
6. Create a branch from dev. In some cases, the branch doesn’t have to dev. Please see the Git work flow for more information.
7. Make sure the pre-commit is working with CPPLinter, Clang, Black, Flake8, etc. This step is very important. 
8. Make commits to the branch that you have created. Please make sure you push your code to the remote branch everyday so that you don’t lose you data. It is okay to push even if you have just one WIP commit. Follow the standards mentioned below to write a commit heading and message.
9. Please follow the coding style mentioned below.
10. Create PR on GitHub. Provide a proper heading and  the JIRA ticket code.  Make sure your heading is clear to a non-programmer. The heading shouldn’t be the name of the branch.
11.  Fill the entire template. Add the version, how the reviewers can run it, what contribution are you making, documentation to be updated, future contribution, etc.
12. Make sure you link the confluence page or documentation to this template.
13. Link the issue to the PR.
14. If your PR is still WIP, covert it into a draft.
15. Update the [README.md](http://README.md) file on the repo. If you have new parameters added to the launch file or run command add them.
16. Test your code on the simulation and the real hardware.
17. Go to the Files Changed tab on the PR and ask yourself if these changes are correct and there is no accidental spacing and types. This is a very important step.
18. When you think you have completed the PR, please add the name of the reviewers on the PR description and convert the PR from draft to ready for review (open, green color).
19. If your reviewers doesn’t review your task on time please ping them and get a estimated time of review. If the PR is critical to be merge inform them the importance. You are responsible for your PR.
20. Once you get comments from your reviewers. Resolve them and add on a comment 'Resolved with commit <commit hash>' and click on Resolve Conversation.
21. Test your code on the simulation and the real hardware.
22. Once you finish resolving all the comments, request for review again. Follow the above four steps until it is completely approved.
23. Once approved, make sure the CI is working properly, tagging is done properly so that the correct version is pushed to dockerhub and the job is finished successfully.
24. Merge the branch.
25. Close the linked issue. 
26. Have a beer.