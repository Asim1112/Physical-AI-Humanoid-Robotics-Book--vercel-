#!/usr/bin/env python3
# Example: Python script to validate URDF models
# Demonstrates working with URDF files programmatically

import xml.etree.ElementTree as ET
from collections import defaultdict
import sys
import os


class URDFValidator:
    """
    A class to validate URDF models and check for common issues.
    Demonstrates parsing and validating URDF files programmatically.
    """

    def __init__(self, urdf_path):
        self.urdf_path = urdf_path
        self.tree = None
        self.root = None
        self.links = {}
        self.joints = {}

    def load_urdf(self):
        """Load and parse the URDF file."""
        try:
            if not os.path.exists(self.urdf_path):
                raise FileNotFoundError(f"URDF file not found: {self.urdf_path}")

            self.tree = ET.parse(self.urdf_path)
            self.root = self.tree.getroot()

            if self.root.tag != 'robot':
                raise ValueError("Invalid URDF: Root element is not 'robot'")

            self.parse_links()
            self.parse_joints()

            return True
        except ET.ParseError as e:
            print(f"XML Parse Error: {e}")
            return False
        except Exception as e:
            print(f"Error loading URDF: {e}")
            return False

    def parse_links(self):
        """Parse all links from the URDF."""
        for link_elem in self.root.findall('link'):
            link_name = link_elem.get('name')
            if link_name:
                self.links[link_name] = {
                    'visual': link_elem.find('visual') is not None,
                    'collision': link_elem.find('collision') is not None,
                    'inertial': link_elem.find('inertial') is not None
                }

    def parse_joints(self):
        """Parse all joints from the URDF."""
        for joint_elem in self.root.findall('joint'):
            joint_name = joint_elem.get('name')
            joint_type = joint_elem.get('type')

            if joint_name and joint_type:
                parent_elem = joint_elem.find('parent')
                child_elem = joint_elem.find('child')

                parent_name = parent_elem.get('link') if parent_elem is not None else None
                child_name = child_elem.get('link') if child_elem is not None else None

                self.joints[joint_name] = {
                    'type': joint_type,
                    'parent': parent_name,
                    'child': child_name
                }

    def validate_structure(self):
        """Validate the overall structure of the URDF."""
        issues = []

        # Check for duplicate link names
        link_names = [name for name in self.links.keys()]
        if len(link_names) != len(set(link_names)):
            issues.append("Duplicate link names found")

        # Check for duplicate joint names
        joint_names = [name for name in self.joints.keys()]
        if len(joint_names) != len(set(joint_names)):
            issues.append("Duplicate joint names found")

        # Check that all joint parents and children exist as links
        for joint_name, joint_data in self.joints.items():
            if joint_data['parent'] not in self.links:
                issues.append(f"Joint '{joint_name}' has non-existent parent link '{joint_data['parent']}'")
            if joint_data['child'] not in self.links:
                issues.append(f"Joint '{joint_name}' has non-existent child link '{joint_data['child']}'")

        # Check for joints that connect a link to itself
        for joint_name, joint_data in self.joints.items():
            if joint_data['parent'] == joint_data['child']:
                issues.append(f"Joint '{joint_name}' connects a link to itself")

        return issues

    def validate_physical_properties(self):
        """Validate physical properties of links."""
        issues = []

        for link_name, link_data in self.links.items():
            # Check for links without visual or collision elements
            if not link_data['visual'] and not link_data['collision']:
                issues.append(f"Link '{link_name}' has neither visual nor collision elements")

            # Check for links without inertial properties (important for simulation)
            if not link_data['inertial']:
                issues.append(f"Link '{link_name}' has no inertial properties (required for physics simulation)")

        return issues

    def validate_joint_limits(self):
        """Validate joint limits where applicable."""
        issues = []

        for joint_name, joint_data in self.joints.items():
            if joint_data['type'] in ['revolute', 'prismatic']:
                limit_elem = self.root.find(f".//joint[@name='{joint_name}']/limit")
                if limit_elem is not None:
                    lower = limit_elem.get('lower')
                    upper = limit_elem.get('upper')

                    if lower is not None and upper is not None:
                        try:
                            lower_val = float(lower)
                            upper_val = float(upper)
                            if lower_val >= upper_val:
                                issues.append(f"Joint '{joint_name}' has invalid limits: lower ({lower_val}) >= upper ({upper_val})")
                        except ValueError:
                            issues.append(f"Joint '{joint_name}' has invalid limit values")

        return issues

    def check_kinematic_chain(self):
        """Check if the kinematic structure is reasonable."""
        issues = []

        # Count how many links have no parent (base links)
        child_links = set()
        for joint_data in self.joints.values():
            if joint_data['child']:
                child_links.add(joint_data['child'])

        all_links = set(self.links.keys())
        base_links = all_links - child_links

        if len(base_links) == 0:
            issues.append("No base link found - all links are children of other links")
        elif len(base_links) > 1:
            issues.append(f"Multiple base links found: {list(base_links)} - consider if this is intentional")

        return issues

    def validate_all(self):
        """Run all validations and return a comprehensive report."""
        if not self.load_urdf():
            return {"valid": False, "errors": [f"Could not load URDF file: {self.urdf_path}"], "warnings": []}

        all_issues = []
        all_issues.extend(self.validate_structure())
        all_issues.extend(self.validate_physical_properties())
        all_issues.extend(self.validate_joint_limits())
        all_issues.extend(self.check_kinematic_chain())

        # Separate errors and warnings
        errors = []
        warnings = []

        for issue in all_issues:
            # For this example, we'll consider structure issues as errors, others as warnings
            if any(keyword in issue.lower() for keyword in ['non-existent', 'duplicate', 'itself', 'no base']):
                errors.append(issue)
            else:
                warnings.append(issue)

        return {
            "valid": len(errors) == 0,
            "errors": errors,
            "warnings": warnings,
            "link_count": len(self.links),
            "joint_count": len(self.joints)
        }

    def print_report(self, report):
        """Print a formatted validation report."""
        print(f"\nURDF Validation Report for: {self.urdf_path}")
        print("=" * 50)
        print(f"Valid: {'YES' if report['valid'] else 'NO'}")
        print(f"Links: {report['link_count']}, Joints: {report['joint_count']}")

        if report['errors']:
            print(f"\nErrors ({len(report['errors'])}):")
            for i, error in enumerate(report['errors'], 1):
                print(f"  {i}. {error}")

        if report['warnings']:
            print(f"\nWarnings ({len(report['warnings'])}):")
            for i, warning in enumerate(report['warnings'], 1):
                print(f"  {i}. {warning}")

        if not report['errors'] and not report['warnings']:
            print("\nNo issues found - URDF appears valid!")

        print("=" * 50)


def main():
    """Main function to run the URDF validator."""
    if len(sys.argv) < 2:
        print("Usage: python3 validate_urdf.py <path_to_urdf_file>")
        print("Example: python3 validate_urdf.py simple_robot.urdf")
        return

    urdf_path = sys.argv[1]
    validator = URDFValidator(urdf_path)
    report = validator.validate_all()
    validator.print_report(report)


if __name__ == '__main__':
    main()