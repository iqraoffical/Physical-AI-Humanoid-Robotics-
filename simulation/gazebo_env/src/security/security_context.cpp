#ifndef SECURITY_CONTEXT_H
#define SECURITY_CONTEXT_H

#include <string>
#include <map>
#include <vector>
#include <memory>
#include <mutex>
#include <chrono>
#include <iostream>

// Forward declarations
class SecurityToken;
class AccessControlList;
class EncryptionManager;

/**
 * @brief Security context for the digital twin system
 * Implements security framework with access control and message encryption
 * Addresses clarifications regarding security implementation in the system
 */
class SecurityContext {
public:
    /**
     * @brief Security level enumeration
     * Defines different security levels for access control
     */
    enum class SecurityLevel {
        UNTRUSTED = 0,      // No access
        AUTHENTICATED = 1,  // Basic authenticated access
        AUTHORIZED = 2,     // Authorized access with permissions
        SECURE = 3,         // Secure communication with encryption
        PRIVILEGED = 4,     // Privileged access with elevated permissions
        ADMIN = 5           // Administrative access to all resources
    };

    /**
     * @brief Constructor
     * Initializes the security context with default settings
     */
    SecurityContext();

    /**
     * @brief Destructor
     * Cleans up security resources
     */
    virtual ~SecurityContext();

    /**
     * @brief Initialize security context with configuration
     * @param config_path Path to security configuration file
     * @return True if initialization successful, false otherwise
     * Implements security configuration from clarifications
     */
    bool Initialize(const std::string& config_path);

    /**
     * @brief Authenticate a user or system component
     * @param credentials Authentication credentials
     * @return Security token if authentication successful, nullptr otherwise
     */
    std::shared_ptr<SecurityToken> Authenticate(const std::string& credentials);

    /**
     * @brief Validate access to a resource
     * @param resource_name Name of the resource to access
     * @param token Security token for the requesting entity
     * @param required_level Minimum security level required
     * @return True if access granted, false otherwise
     */
    bool ValidateAccess(const std::string& resource_name, 
                       const std::shared_ptr<SecurityToken>& token, 
                       SecurityLevel required_level);

    /**
     * @brief Encrypt data using current security context
     * @param data Data to encrypt
     * @return Encrypted data
     */
    std::string Encrypt(const std::string& data);

    /**
     * @brief Decrypt data using current security context
     * @param encrypted_data Data to decrypt
     * @return Decrypted data
     */
    std::string Decrypt(const std::string& encrypted_data);

    /**
     * @brief Apply security policy to a message
     * @param message Message to secure
     * @param token Security token for the sender
     * @return Secured message
     */
    std::string ApplySecurityPolicy(const std::string& message, 
                                   const std::shared_ptr<SecurityToken>& token);

    /**
     * @brief Check if security context is properly initialized
     * @return True if security context is ready for use, false otherwise
     */
    bool IsInitialized() const;

    /**
     * @brief Get current security level
     * @return Current security level
     */
    SecurityLevel GetSecurityLevel() const;

    /**
     * @brief Update security configuration
     * @param new_config_path Path to new security configuration
     * @return True if update successful, false otherwise
     */
    bool UpdateConfiguration(const std::string& new_config_path);

    /**
     * @brief Generate audit log entry
     * @param event_type Type of security event
     * @param details Details about the event
     * @return Audit log entry
     */
    std::string GenerateAuditEntry(const std::string& event_type, 
                                  const std::string& details);

    /**
     * @brief Validate certificate against trust store
     * @param certificate Certificate to validate
     * @return True if certificate is valid, false otherwise
     */
    bool ValidateCertificate(const std::string& certificate);

    /**
     * @brief Manage key rotation for encryption
     * Implements key rotation requirement from clarifications
     */
    void RotateEncryptionKeys();

    /**
     * @brief Check if a user has permission for specific action
     * @param user_id User identifier
     * @param action Action to perform
     * @return True if user has permission, false otherwise
     */
    bool HasPermission(const std::string& user_id, const std::string& action);

private:
    /// Security level of the current context
    SecurityLevel current_level;

    /// Access control list
    std::unique_ptr<AccessControlList> acl;

    /// Encryption manager
    std::unique_ptr<EncryptionManager> encryption_mgr;

    /// Security tokens cache
    std::map<std::string, std::shared_ptr<SecurityToken>> token_cache;

    /// Trust store for certificates
    std::vector<std::string> trust_store;

    /// Configuration file path
    std::string config_path;

    /// Encryption keys
    std::map<std::string, std::string> encryption_keys;

    /// Last key rotation time
    std::chrono::steady_clock::time_point last_key_rotation;

    /// Security context mutex for thread safety
    mutable std::mutex security_mutex;

    /// Whether security context is initialized
    bool initialized;

    /// Security policies
    std::map<std::string, std::string> security_policies;

    /**
     * @brief Initialize access control list
     * @return True if initialization successful, false otherwise
     */
    bool InitializeACL();

    /**
     * @brief Initialize encryption manager
     * @return True if initialization successful, false otherwise
     */
    bool InitializeEncryption();

    /**
     * @brief Load certificates from trust store
     * @param store_path Path to trust store
     * @return True if certificates loaded successfully, false otherwise
     */
    bool LoadCertificates(const std::string& store_path);

    /**
     * @brief Validate configuration settings
     * @param config_path Path to configuration file
     * @return True if configuration is valid, false otherwise
     */
    bool ValidateConfiguration(const std::string& config_path);
};

/**
 * @brief Security token class
 * Represents an authenticated and authorized entity in the system
 */
class SecurityToken {
public:
    /// Token ID
    std::string token_id;

    /// User ID
    std::string user_id;

    /// Security level
    SecurityContext::SecurityLevel security_level;

    /// Expiration time
    std::chrono::steady_clock::time_point expiration_time;

    /// Permissions associated with the token
    std::vector<std::string> permissions;

    /// Constructor
    SecurityToken(const std::string& id, 
                  const std::string& user, 
                  SecurityContext::SecurityLevel level,
                  std::chrono::seconds validity);

    /// Check if token is expired
    bool IsExpired() const;

    /// Check if token has specific permission
    bool HasPermission(const std::string& permission) const;
};

/**
 * @brief Access Control List implementation
 * Manages permissions and access rights for different entities
 */
class AccessControlList {
public:
    /**
     * @brief Add permission for a user/entity
     * @param user_id User identifier
     * @param resource Resource name
     * @param permission Permission to grant
     * @return True if permission granted successfully, false otherwise
     */
    bool GrantPermission(const std::string& user_id, 
                        const std::string& resource, 
                        const std::string& permission);

    /**
     * @brief Revoke permission for a user/entity
     * @param user_id User identifier
     * @param resource Resource name
     * @param permission Permission to revoke
     * @return True if permission revoked successfully, false otherwise
     */
    bool RevokePermission(const std::string& user_id, 
                         const std::string& resource, 
                         const std::string& permission);

    /**
     * @brief Check if user has permission for resource
     * @param user_id User identifier
     * @param resource Resource name
     * @param permission Permission to check
     * @return True if user has permission, false otherwise
     */
    bool HasPermission(const std::string& user_id, 
                      const std::string& resource, 
                      const std::string& permission) const;

    /**
     * @brief Get all permissions for a user
     * @param user_id User identifier
     * @return Vector of permissions
     */
    std::vector<std::string> GetUserPermissions(const std::string& user_id) const;

private:
    /// Permission map: user -> resource -> permissions
    std::map<std::string, std::map<std::string, std::vector<std::string>>> permissions_map;

    /// Mutex for thread safety
    mutable std::mutex acl_mutex;
};

/**
 * @brief Encryption Manager
 * Handles encryption and decryption operations
 */
class EncryptionManager {
public:
    /**
     * @brief Constructor
     * @param algorithm Encryption algorithm to use
     */
    EncryptionManager(const std::string& algorithm = "AES-256-GCM");

    /**
     * @brief Encrypt data
     * @param data Data to encrypt
     * @param key Encryption key
     * @return Encrypted data
     */
    std::string Encrypt(const std::string& data, const std::string& key);

    /**
     * @brief Decrypt data
     * @param encrypted_data Data to decrypt
     * @param key Encryption key
     * @return Decrypted data
     */
    std::string Decrypt(const std::string& encrypted_data, const std::string& key);

    /**
     * @brief Generate encryption key
     * @return Generated key
     */
    std::string GenerateKey();

    /**
     * @brief Rotate encryption key
     * @return New encryption key
     */
    std::string RotateKey();

private:
    /// Encryption algorithm
    std::string algorithm;

    /// Current encryption key
    std::string current_key;

    /// Previous encryption key (for transition period)
    std::string previous_key;

    /// Mutex for thread safety
    mutable std::mutex encryption_mutex;
};

// Inline method implementations
inline bool SecurityContext::IsInitialized() const {
    return initialized;
}

inline SecurityContext::SecurityLevel SecurityContext::GetSecurityLevel() const {
    return current_level;
}

inline bool SecurityToken::IsExpired() const {
    return std::chrono::steady_clock::now() > expiration_time;
}

inline bool SecurityToken::HasPermission(const std::string& perm) const {
    return std::find(permissions.begin(), permissions.end(), perm) != permissions.end();
}

#endif /* SECURITY_CONTEXT_H */