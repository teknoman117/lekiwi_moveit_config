#!/usr/bin/env python3
"""Compute the SRDF <disable_collisions> pairs of a robot, like the MoveIt Setup Assistant.

A pair of links is disabled when:
  Adjacent  a joint connects them (skipping links without collision geometry);
  Default   they are in contact with every joint at its default (zero) position;
  Always    they are in contact in every random sample;
  Never     neither moves relative to the base (neither is at or below --moving-root), and they
            were never in contact in the samples. Pairs with a moving link are never disabled
            this way, so the arm stays checked against everything else.

Usage (sourced workspace; needs moveit_py, ros-jazzy-moveit-py):
  compute_disabled_collisions.py MODEL SRDF [--samples 2000] [--moving-root shoulder_link]
                                 [--new-only]

MODEL and SRDF are files, or xacro files (*.xacro, processed with xacro). With --new-only, only
the pairs that SRDF does not disable yet are printed: for a robot that adds parts to the LeKiwi,
pass its full model and its SRDF (lekiwi_semantic plus its own pairs) and paste the output into it.
"""

import argparse
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import Counter


def expand(path):
    """The file itself, or a temporary file with its xacro output."""
    if not path.endswith('.xacro'):
        return path
    result = subprocess.run(['xacro', path], capture_output=True, text=True)
    if result.returncode != 0:
        sys.exit(f'xacro {path} failed:\n{result.stderr}')
    out = tempfile.NamedTemporaryFile('w', suffix='.xml', delete=False)
    out.write(result.stdout)
    out.close()
    return out.name


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('model')
    parser.add_argument('srdf')
    parser.add_argument('--samples', type=int, default=2000)
    parser.add_argument('--moving-root', default='shoulder_link',
                        help='the first link that moves relative to the base (default: the '
                             "SO-101's shoulder_link)")
    parser.add_argument('--new-only', action='store_true',
                        help='print only the pairs the SRDF does not disable yet')
    args = parser.parse_args()

    from moveit.core.collision_detection import (AllowedCollisionMatrix, CollisionRequest,
                                                 CollisionResult)
    from moveit.core.planning_scene import PlanningScene
    from moveit.core.robot_model import RobotModel
    from moveit.core.robot_state import RobotState

    urdf, srdf = expand(args.model), expand(args.srdf)
    model = RobotModel(urdf_xml_path=urdf, srdf_xml_path=srdf)
    scene = PlanningScene(model)

    root = ET.parse(urdf).getroot()
    geometric = sorted(l.get('name') for l in root.findall('link')
                       if l.find('collision') is not None)
    parent = {j.find('child').get('link'): j.find('parent').get('link')
              for j in root.findall('joint')}

    def geometric_parent(link):
        p = parent.get(link)
        while p is not None and p not in geometric:
            p = parent.get(p)
        return p

    def moves(link):
        while link is not None:
            if link == args.moving_root:
                return True
            link = parent.get(link)
        return False

    pairs = [(a, b) for i, a in enumerate(geometric) for b in geometric[i + 1:]]
    adjacent = {tuple(sorted((l, geometric_parent(l)))) for l in geometric if geometric_parent(l)}
    check_all = AllowedCollisionMatrix(geometric, False)
    pair_acm = {}
    for a, b in pairs:
        m = AllowedCollisionMatrix(geometric, True)
        m.set_entry(a, b, False)
        pair_acm[(a, b)] = m

    def collides(state, acm):
        res = CollisionResult()
        scene.check_self_collision(CollisionRequest(), res, state, acm)
        return res.collision

    def colliding(state):
        if not collides(state, check_all):
            return set()
        return {p for p in pairs if collides(state, pair_acm[p])}

    state = RobotState(model)
    state.set_to_default_values()
    state.update()
    default = colliding(state) - adjacent
    counts = Counter()
    for _ in range(args.samples):
        state.set_to_random_positions()
        state.update()
        counts.update(colliding(state))
    always = {p for p, n in counts.items() if n == args.samples} - adjacent - default
    never = {p for p in pairs if not moves(p[0]) and not moves(p[1]) and counts[p] == 0}
    never -= adjacent | default | always

    reasons = {**{p: 'Never' for p in never}, **{p: 'Adjacent' for p in adjacent},
               **{p: 'Default' for p in default}, **{p: 'Always' for p in always}}
    if args.new_only:
        existing = {tuple(sorted((d.get('link1'), d.get('link2'))))
                    for d in ET.parse(srdf).getroot().findall('disable_collisions')}
        reasons = {p: r for p, r in reasons.items() if p not in existing}
    for p in sorted(reasons):
        print(f'<disable_collisions link1="{p[0]}" link2="{p[1]}" reason="{reasons[p]}"/>')
    checked = sorted(p for p in pairs if p not in reasons)
    print(f'# {len(geometric)} links with collision geometry, {len(pairs)} pairs; '
          f'{len(reasons)} printed, {len(checked)} stay checked', file=sys.stderr)


if __name__ == '__main__':
    main()
