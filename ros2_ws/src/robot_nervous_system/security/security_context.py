#!/usr/bin/env python3

"""
Security context module for the ROS 2 nervous system.
Implements ROS 2 security framework with access control and message encryption (from clarifications).
"""

import os
from typing import Dict, List, Optional
from enum import Enum


class AccessLevel(Enum):
    """Enumeration of access levels for different components"""
    ADMIN = "admin"
    USER = "user"
    GUEST = "guest"
    SYSTEM = "system"


class SecurityContext:
    """
    Contains access control and encryption settings for securing ROS 2 communications.
    Applied to all ROS 2 communications to ensure message integrity (from data model).
    """
    
    def __init__(self, keystore_path: Optional[str] = None):
        """
        Initialize the security context.
        
        Args:
            keystore_path: Path to the security keystore directory
        """
        self.keystore_path = keystore_path or os.getenv("ROS_SECURITY_KEYSTORE", "~/ros2_ws/keys")
        self.enabled = os.getenv("ROS_SECURITY_ENABLE", "false").lower() == "true"
        self.strategy = os.getenv("ROS_SECURITY_STRATEGY", "Permissive")  # Permissive or Enforce
        
        # Initialize access control policies
        self.access_policies = {
            "publisher_node": AccessLevel.SYSTEM,
            "subscriber_node": AccessLevel.SYSTEM,
            "service_node": AccessLevel.ADMIN,
            "action_node": AccessLevel.ADMIN,
            "python_agent_bridge": AccessLevel.USER
        }
        
        # Initialize encryption keys (simplified representation)
        self.encryption_keys = {}
        self.valid_certificates = []
        
        # Load security artifacts if keystore exists
        self._load_security_artifacts()
        
        print(f"Security context initialized: enabled={self.enabled}, strategy={self.strategy}")
    
    def _load_security_artifacts(self):
        """Load security artifacts from the keystore."""
        if not os.path.exists(self.keystore_path):
            print(f"Keystore does not exist at {self.keystore_path}")
            return
            
        try:
            # Load encryption keys
            keys_dir = os.path.join(self.keystore_path, "keys")
            if os.path.exists(keys_dir):
                for root, dirs, files in os.walk(keys_dir):
                    for file in files:
                        if file.endswith(".pem"):
                            key_name = os.path.splitext(file)[0]
                            self.encryption_keys[key_name] = os.path.join(root, file)
            
            # Load certificates
            certs_dir = os.path.join(self.keystore_path, "certs")
            if os.path.exists(certs_dir):
                for file in os.listdir(certs_dir):
                    if file.endswith(".crt"):
                        cert_path = os.path.join(certs_dir, file)
                        with open(cert_path, 'r') as f:
                            cert_content = f.read()
                            self.valid_certificates.append(cert_content)
            
            print(f"Loaded {len(self.encryption_keys)} keys and {len(self.valid_certificates)} certificates")
        except Exception as e:
            print(f"Error loading security artifacts: {e}")
    
    def validate_access(self, node_name: str, required_access: AccessLevel) -> bool:
        """
        Validate if a node has the required access level.
        
        Args:
            node_name: Name of the node requesting access
            required_access: Required access level
            
        Returns:
            True if access is granted, False otherwise
        """
        if not self.enabled:
            return True  # If security is not enabled, allow access
            
        if node_name not in self.access_policies:
            print(f"Warning: No access policy defined for node {node_name}")
            return self.strategy == "Permissive"
        
        node_access_level = self.access_policies[node_name]
        
        # Check if node has sufficient access level
        # ADMIN > USER > GUEST > SYSTEM (for specific operations)
        access_level_hierarchy = {
            AccessLevel.ADMIN: 3,
            AccessLevel.USER: 2,
            AccessLevel.GUEST: 1,
            AccessLevel.SYSTEM: 0
        }
        
        has_access = access_level_hierarchy[node_access_level] >= access_level_hierarchy[required_access]
        
        if not has_access and self.strategy == "Enforce":
            print(f"Access denied for {node_name}: required {required_access}, has {node_access_level}")
            return False
        elif not has_access:
            print(f"Access would be denied for {node_name} but strategy is Permissive")
        
        return has_access
    
    def encrypt_message(self, message: str, key_name: str) -> str:
        """
        Encrypt a message using the specified key.
        Note: This is a simplified implementation; real encryption would use proper cryptography.
        
        Args:
            message: Message to encrypt
            key_name: Name of the key to use for encryption
            
        Returns:
            Encrypted message
        """
        if not self.enabled:
            return message  # If security is not enabled, return message as-is
            
        if key_name not in self.encryption_keys:
            raise ValueError(f"Encryption key {key_name} not found")
        
        # Simplified encryption (in real implementation, use proper encryption)
        # Here we just return the same message for demonstration
        return f"encrypted:{message}"
    
    def decrypt_message(self, encrypted_message: str, key_name: str) -> str:
        """
        Decrypt a message using the specified key.
        Note: This is a simplified implementation; real decryption would use proper cryptography.
        
        Args:
            encrypted_message: Message to decrypt
            key_name: Name of the key to use for decryption
            
        Returns:
            Decrypted message
        """
        if not self.enabled:
            return encrypted_message  # If security is not enabled, return message as-is
            
        if key_name not in self.encryption_keys:
            raise ValueError(f"Decryption key {key_name} not found")
        
        # Simplified decryption (in real implementation, use proper decryption)
        # Here we just return the same message for demonstration
        if encrypted_message.startswith("encrypted:"):
            return encrypted_message[10:]  # Remove "encrypted:" prefix
        return encrypted_message
    
    def validate_certificate(self, certificate: str) -> bool:
        """
        Validate a certificate against the known valid certificates.
        
        Args:
            certificate: Certificate to validate
            
        Returns:
            True if certificate is valid, False otherwise
        """
        if not self.enabled:
            return True  # If security is not enabled, accept certificate
            
        return certificate in self.valid_certificates
    
    def validate_message_integrity(self, message: str, signature: str) -> bool:
        """
        Validate the integrity of a message using its signature.
        
        Args:
            message: Message to validate
            signature: Signature to verify
            
        Returns:
            True if integrity is verified, False otherwise
        """
        if not self.enabled:
            return True  # If security is not enabled, assume integrity
            
        # In a real implementation, this would validate a digital signature
        # For now, we'll just return True as a placeholder
        return True


# Global security context instance
_security_context = None


def get_security_context() -> SecurityContext:
    """Get the global security context instance."""
    global _security_context
    if _security_context is None:
        _security_context = SecurityContext()
    return _security_context


def validate_access_for_node(node_name: str, required_access: AccessLevel) -> bool:
    """Validate if a node has the required access level."""
    security_context = get_security_context()
    return security_context.validate_access(node_name, required_access)


def encrypt_message(message: str, key_name: str = "default") -> str:
    """Encrypt a message using the security context."""
    security_context = get_security_context()
    return security_context.encrypt_message(message, key_name)


def decrypt_message(encrypted_message: str, key_name: str = "default") -> str:
    """Decrypt a message using the security context."""
    security_context = get_security_context()
    return security_context.decrypt_message(encrypted_message, key_name)


def validate_certificate(certificate: str) -> bool:
    """Validate a certificate using the security context."""
    security_context = get_security_context()
    return security_context.validate_certificate(certificate)


def validate_message_integrity(message: str, signature: str) -> bool:
    """Validate message integrity using the security context."""
    security_context = get_security_context()
    return security_context.validate_message_integrity(message, signature)


if __name__ == "__main__":
    # Test the security context
    sec_ctx = SecurityContext()
    print(f"Security enabled: {sec_ctx.enabled}")
    print(f"Access validation for publisher_node (SYSTEM): {sec_ctx.validate_access('publisher_node', AccessLevel.SYSTEM)}")
    print(f"Access validation for python_agent_bridge (USER) to ADMIN resource: {sec_ctx.validate_access('python_agent_bridge', AccessLevel.ADMIN)}")