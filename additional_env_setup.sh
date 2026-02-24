#!/bin/bash

set -e  # stop on any error

export WORKSPACE=robot_ws
export ROS_DISTRO=humble
WORKSPACE_SRC=~/${WORKSPACE}/src

# ============================================================
# Helper: clone or update a repo
# ============================================================
clone_or_update() {
    local REPO_URL=$1
    local BRANCH=$2
    local REPO_NAME=$(basename $REPO_URL .git)

    if [ -d "$WORKSPACE_SRC/$REPO_NAME" ]; then
        echo ">>> $REPO_NAME already exists, pulling latest..."
        cd $WORKSPACE_SRC/$REPO_NAME
        git pull origin $BRANCH
    else
        echo ">>> Cloning $REPO_NAME..."
        cd $WORKSPACE_SRC
        git clone -b $BRANCH $REPO_URL
    fi
}

# ============================================================
# Helper: apply a PR, skip if already applied
# ============================================================
apply_pr() {
    local REPO_NAME=$1
    local PR_NUMBER=$2
    local REPO_URL=$3

    cd $WORKSPACE_SRC/$REPO_NAME

    # Check if PR branch already exists
    if git branch | grep -q "pr-$PR_NUMBER"; then
        echo ">>> PR #$PR_NUMBER already applied to $REPO_NAME, skipping..."
        return 0
    fi

    echo ">>> Fetching PR #$PR_NUMBER for $REPO_NAME..."
    git fetch origin pull/$PR_NUMBER/head:pr-$PR_NUMBER
    if [ $? -ne 0 ]; then
        echo ""
        echo "*** ERROR: Could not fetch PR #$PR_NUMBER for $REPO_NAME"
        echo "*** ACTION REQUIRED: Check if PR still exists at:"
        echo "*** $REPO_URL/pull/$PR_NUMBER"
        return 1
    fi

    echo ">>> Merging PR #$PR_NUMBER into $ROS_DISTRO..."
    git merge pr-$PR_NUMBER --no-edit
    if [ $? -ne 0 ]; then
        echo ""
        echo "*** WARNING: PR #$PR_NUMBER failed to merge cleanly into $REPO_NAME"
        echo "*** The $ROS_DISTRO branch may have changed since the PR was opened."
        echo "*** ACTION REQUIRED:"
        echo "***   1. Check PR status at: $REPO_URL/pull/$PR_NUMBER"
        echo "***   2. Resolve conflicts manually in: $WORKSPACE_SRC/$REPO_NAME"
        echo "***   3. Check if PR has been superseded by a newer commit"
        git merge --abort 2>/dev/null
        return 1
    fi

    echo ">>> Successfully applied PR #$PR_NUMBER to $REPO_NAME"
    echo ">>> Applied commit: $(git log --oneline -1)"
}

# ============================================================
# Clone or update repos
# ============================================================
clone_or_update https://github.com/ROBOTIS-GIT/DynamixelSDK.git $ROS_DISTRO
clone_or_update https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git $ROS_DISTRO
clone_or_update https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git $ROS_DISTRO

# ============================================================
# Apply PRs
# ============================================================
apply_pr dynamixel_hardware_interface 107 https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface
apply_pr dynamixel_interfaces 10 https://github.com/ROBOTIS-GIT/dynamixel_interfaces

echo ">>> Installing dependencies via rosdep..."
cd ~/${WORKSPACE}
rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO

echo ">>> setup Rviz2 environment..."

mkdir -p ~/.rviz2
cp ${WORKSPACE_SRC}/writing_robot_description/rviz/persistent_settings ~/.rviz2
cp ${WORKSPACE_SRC}/writing_robot_description/rviz/default.rviz ~/.rviz2

echo ""
echo "----------------------------------------------"
echo "Setup complete. Check above for any warnings."
echo "----------------------------------------------"
