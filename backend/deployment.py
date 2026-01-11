"""Vercel deployment functions for Docusaurus frontend."""

import logging
import subprocess
import sys
from pathlib import Path
from typing import Optional, Tuple

import requests

logger = logging.getLogger("rag_ingestion")


# ==============================================================================
# T015: Deploy to Vercel
# ==============================================================================

def deploy_to_vercel(frontend_path: str = "../frontend", production: bool = True) -> Optional[str]:
    """
    Deploy Docusaurus frontend to Vercel using CLI.

    Args:
        frontend_path: Path to frontend directory (relative to backend/)
        production: Deploy to production (True) or preview (False)

    Returns:
        Deployment URL if successful, None if failed

    Raises:
        RuntimeError: If Vercel CLI is not found or deployment fails
    """
    frontend_dir = Path(__file__).parent / frontend_path

    if not frontend_dir.exists():
        raise FileNotFoundError(f"Frontend directory not found: {frontend_dir}")

    logger.info(f"Deploying frontend from {frontend_dir} to Vercel...")

    # Check if Vercel CLI is installed
    try:
        result = subprocess.run(
            ["vercel", "--version"],
            capture_output=True,
            text=True,
            timeout=10
        )
        logger.debug(f"Vercel CLI version: {result.stdout.strip()}")
    except FileNotFoundError:
        raise RuntimeError(
            "Vercel CLI not found. Install with: npm install -g vercel\n"
            "Then authenticate with: vercel login"
        )
    except subprocess.TimeoutExpired:
        raise RuntimeError("Vercel CLI check timed out")

    # Build command
    cmd = ["vercel", "--yes"]  # --yes for non-interactive
    if production:
        cmd.append("--prod")

    try:
        logger.info(f"Running: {' '.join(cmd)}")
        result = subprocess.run(
            cmd,
            cwd=str(frontend_dir),
            capture_output=True,
            text=True,
            timeout=300  # 5 minutes max
        )

        if result.returncode != 0:
            logger.error(f"Vercel deployment failed: {result.stderr}")
            raise RuntimeError(f"Deployment failed with exit code {result.returncode}")

        # Extract URL from output
        output_lines = result.stdout.strip().split('\n')
        deployment_url = None

        for line in output_lines:
            # Vercel outputs URL in various formats, look for https://
            if "https://" in line:
                # Extract URL (typically on its own line or after "Deployed to")
                parts = line.split()
                for part in parts:
                    if part.startswith("https://"):
                        deployment_url = part.strip()
                        break

        if deployment_url:
            logger.info(f"[OK] Deployment successful: {deployment_url}")
            return deployment_url
        else:
            logger.warning("Deployment completed but URL not found in output")
            logger.debug(f"Full output: {result.stdout}")
            return None

    except subprocess.TimeoutExpired:
        logger.error("Deployment timed out after 5 minutes")
        raise RuntimeError("Deployment timeout exceeded")
    except Exception as e:
        logger.error(f"Deployment error: {e}")
        raise


# ==============================================================================
# T016: Verify Deployment
# ==============================================================================

def verify_deployment(url: str, timeout: int = 30) -> Tuple[bool, Optional[str]]:
    """
    Verify deployment by checking URL accessibility.

    Args:
        url: Deployment URL to verify
        timeout: Request timeout in seconds

    Returns:
        Tuple of (success: bool, error_message: Optional[str])

    Example:
        success, error = verify_deployment("https://my-app.vercel.app")
        if success:
            print("Deployment verified!")
        else:
            print(f"Verification failed: {error}")
    """
    logger.info(f"Verifying deployment at {url}...")

    try:
        response = requests.get(url, timeout=timeout, allow_redirects=True)

        if response.status_code == 200:
            # Check if response looks like HTML (not an error page)
            content_type = response.headers.get('content-type', '')
            if 'text/html' in content_type:
                logger.info(f"[OK] Deployment verified: HTTP {response.status_code}")

                # Additional checks
                content_lower = response.text.lower()
                checks = {
                    "contains_docusaurus": "docusaurus" in content_lower,
                    "contains_html": "<html" in content_lower,
                    "not_error_page": "404" not in content_lower and "error" not in content_lower[:1000]
                }

                logger.debug(f"Content checks: {checks}")

                if not checks["contains_html"]:
                    return False, "Response is not valid HTML"

                return True, None
            else:
                return False, f"Unexpected content type: {content_type}"
        else:
            return False, f"HTTP {response.status_code}: {response.reason}"

    except requests.exceptions.Timeout:
        return False, f"Request timed out after {timeout}s"
    except requests.exceptions.ConnectionError as e:
        return False, f"Connection error: {e}"
    except requests.exceptions.RequestException as e:
        return False, f"Request failed: {e}"
    except Exception as e:
        return False, f"Unexpected error: {e}"


def check_deployment_features(url: str) -> dict:
    """
    Check specific Docusaurus features on deployed site.

    Args:
        url: Deployment URL

    Returns:
        Dictionary of feature checks (navigation, search, etc.)
    """
    logger.info("Checking deployment features...")

    features = {
        "homepage_accessible": False,
        "has_navigation": False,
        "has_search": False,
        "status_code": None
    }

    try:
        response = requests.get(url, timeout=30)
        features["status_code"] = response.status_code

        if response.status_code == 200:
            content = response.text.lower()

            # Check for Docusaurus navigation
            features["has_navigation"] = (
                "navbar" in content or
                "sidebar" in content or
                "menu" in content
            )

            # Check for search functionality
            features["has_search"] = (
                "search" in content or
                "algolia" in content
            )

            features["homepage_accessible"] = True

        logger.info(f"Feature checks: {features}")

    except Exception as e:
        logger.warning(f"Feature check failed: {e}")

    return features
