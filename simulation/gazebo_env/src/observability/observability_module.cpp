#ifndef OBSERVABILITY_MODULE_H
#define OBSERVABILITY_MODULE_H

#include <string>
#include <map>
#include <vector>
#include <memory>
#include <mutex>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>

// Forward declarations
class MetricsCollector;
class Logger;
class Tracer;

/**
 * @brief Observability data structure
 * Contains logging, metrics, and tracing information for system monitoring
 * Implements observability requirements from clarifications
 */
struct ObservabilityData {
    std::string log_level;                    ///< Severity level of the log
    builtin_interfaces::msg::Time timestamp;  ///< Time of the event (ros time)
    std::string source_node;                  ///< Name of the node generating the data
    std::string message;                      ///< The log message
    std::map<std::string, double> metrics;    ///< Key-value pairs of performance metrics
    std::string trace_id;                     ///< Identifier for tracing requests across nodes
    std::string span_id;                      ///< Identifier for specific operation within trace
    std::string parent_span_id;               ///< Parent span in the trace hierarchy
};

/**
 * @brief Observability module for the digital twin system
 * Implements comprehensive logging, metrics collection, and tracing as specified in the clarifications
 * Provides ROS 2 standard logging with custom metrics and basic tracing for simulation events
 */
class ObservabilityModule {
public:
    /**
     * @brief Log level enumeration
     * Defines different levels of logging severity
     */
    enum class LogLevel {
        DEBUG = 0,
        INFO = 1,
        WARN = 2,
        ERROR = 3,
        FATAL = 4
    };

    /**
     * @brief Constructor
     * Initializes the observability module with default settings
     */
    ObservabilityModule();

    /**
     * @brief Destructor
     * Cleans up observability resources
     */
    virtual ~ObservabilityModule();

    /**
     * @brief Initialize observability module with configuration
     * @param config_path Path to observability configuration file
     * @return True if initialization successful, false otherwise
     */
    bool Initialize(const std::string& config_path);

    /**
     * @brief Log a message with specified level
     * @param level Log severity level
     * @param message Log message to record
     * @param source_file Source file where log originated (optional)
     * @param line_number Line number in source file (optional)
     */
    void Log(LogLevel level, 
             const std::string& message, 
             const std::string& source_file = "", 
             int line_number = 0);

    /**
     * @brief Log a message with additional context
     * @param level Log severity level
     * @param message Log message to record
     * @param context Additional contextual information
     * @param source_file Source file where log originated (optional)
     * @param line_number Line number in source file (optional)
     */
    void LogWithContext(LogLevel level, 
                       const std::string& message, 
                       const std::map<std::string, std::string>& context,
                       const std::string& source_file = "", 
                       int line_number = 0);

    /**
     * @brief Add a metric value for collection
     * @param metric_name Name of the metric
     * @param value Value of the metric
     * @param tags Tags to associate with the metric (optional)
     */
    void AddMetric(const std::string& metric_name, 
                   double value, 
                   const std::map<std::string, std::string>& tags = {});

    /**
     * @brief Start a trace span for performance monitoring
     * @param operation_name Name of the operation being traced
     * @param parent_span_id ID of parent span (empty if root span)
     * @return Span ID for the newly created span
     */
    std::string StartSpan(const std::string& operation_name, 
                         const std::string& parent_span_id = "");

    /**
     * @brief End a trace span
     * @param span_id ID of the span to end
     * @param status Status of the operation (optional)
     */
    void EndSpan(const std::string& span_id, 
                const std::string& status = "OK");

    /**
     * @brief Record an event within a span
     * @param span_id ID of the span containing the event
     * @param event_name Name of the event
     * @param attributes Attributes associated with the event (optional)
     */
    void RecordEvent(const std::string& span_id,
                    const std::string& event_name,
                    const std::map<std::string, std::string>& attributes = {});

    /**
     * @brief Get collected metrics
     * @return Map of metric names to their values
     */
    std::map<std::string, std::vector<double>> GetMetrics() const;

    /**
     * @brief Export metrics in Prometheus format
     * @return Metrics in Prometheus text format
     */
    std::string ExportMetricsPrometheus() const;

    /**
     * @brief Check if observability module is initialized
     * @return True if ready for use, false otherwise
     */
    bool IsInitialized() const;

    /**
     * @brief Set log level threshold
     * @param level Minimum level to log
     */
    void SetLogLevel(LogLevel level);

    /**
     * @brief Flush all buffered logs
     */
    void FlushLogs();

    /**
     * @brief Get current log level
     * @return Current log level threshold
     */
    LogLevel GetLogLevel() const;

    /**
     * @brief Add custom field to all log entries
     * @param key Key for the custom field
     * @param value Value for the custom field
     */
    void AddCustomField(const std::string& key, const std::string& value);

    /**
     * @brief Enable/disable metric collection
     * @param enabled Whether metric collection should be enabled
     */
    void SetMetricsEnabled(bool enabled);

    /**
     * @brief Enable/disable tracing
     * @param enabled Whether tracing should be enabled
     */
    void SetTracingEnabled(bool enabled);

    /**
     * @brief Record performance counter
     * @param counter_name Name of the performance counter
     * @param value Value to record
     */
    void RecordPerformanceCounter(const std::string& counter_name, double value);

    /**
     * @brief Get performance counter values
     * @param counter_name Name of the counter to retrieve
     * @return Vector of recorded values
     */
    std::vector<double> GetPerformanceCounterValues(const std::string& counter_name) const;

    /**
     * @brief Generate system health report
     * @return Health report as string
     */
    std::string GenerateHealthReport() const;

private:
    /// Current log level threshold
    LogLevel current_log_level;

    /// Whether metrics collection is enabled
    bool metrics_enabled;

    /// Whether tracing is enabled
    bool tracing_enabled;

    /// Metrics collector instance
    std::unique_ptr<MetricsCollector> metrics_collector;

    /// Logger instance
    std::unique_ptr<Logger> logger;

    /// Tracer instance
    std::unique_ptr<Tracer> tracer;

    /// Custom fields to add to all logs
    std::map<std::string, std::string> custom_fields;

    /// Configuration file path
    std::string config_path;

    /// Whether observability module is initialized
    bool initialized;

    /// Mutex for thread safety
    mutable std::mutex observability_mutex;

    /**
     * @brief Initialize metrics collector
     * @return True if initialization successful, false otherwise
     */
    bool InitializeMetricsCollector();

    /**
     * @brief Initialize logger
     * @return True if initialization successful, false otherwise
     */
    bool InitializeLogger();

    /**
     * @brief Initialize tracer
     * @return True if initialization successful, false otherwise
     */
    bool InitializeTracer();

    /**
     * @brief Validate configuration settings
     * @param config_path Path to configuration file
     * @return True if configuration is valid, false otherwise
     */
    bool ValidateConfiguration(const std::string& config_path);
};

/**
 * @brief Logger implementation
 * Handles logging with ROS 2 compatible format and multiple output destinations
 */
class Logger {
public:
    /**
     * @brief Constructor
     * @param log_level Default log level
     */
    Logger(ObservabilityModule::LogLevel log_level = ObservabilityModule::LogLevel::INFO);

    /**
     * @brief Log a message
     * @param level Log level
     * @param message Message to log
     * @param file Source file
     * @param line Source line
     * @param function Source function
     */
    void Log(ObservabilityModule::LogLevel level,
             const std::string& message,
             const std::string& file = "",
             int line = 0,
             const std::string& function = "");

    /**
     * @brief Set output destinations (console, file, etc.)
     * @param console_output Whether to output to console
     * @param file_output Path to log file (empty for no file output)
     */
    void SetOutputDestinations(bool console_output, const std::string& file_output = "");

    /**
     * @brief Set log format
     * @param format Log format string
     */
    void SetFormat(const std::string& format);

    /**
     * @brief Flush log buffers
     */
    void Flush();

    /**
     * @brief Add custom field to logs
     * @param key Field key
     * @param value Field value
     */
    void AddCustomField(const std::string& key, const std::string& value);

private:
    /// Current log level
    ObservabilityModule::LogLevel current_level;

    /// Console output enabled
    bool console_enabled;

    /// File output path
    std::string file_path;

    /// Output file stream
    std::ofstream output_file;

    /// Log format string
    std::string log_format;

    /// Custom fields to include in logs
    std::map<std::string, std::string> custom_fields;

    /// Mutex for thread safety
    mutable std::mutex log_mutex;

    /**
     * @brief Format a log message according to the log format
     * @param level Log level
     * @param message Log message
     * @param file Source file
     * @param line Source line
     * @param function Source function
     * @return Formatted log message
     */
    std::string FormatMessage(ObservabilityModule::LogLevel level,
                             const std::string& message,
                             const std::string& file,
                             int line,
                             const std::string& function) const;
};

/**
 * @brief Metrics Collector
 * Collects and manages performance metrics
 */
class MetricsCollector {
public:
    /**
     * @brief Constructor
     */
    MetricsCollector();

    /**
     * @brief Add a metric value
     * @param name Metric name
     * @param value Metric value
     * @param tags Tags associated with the metric
     */
    void AddMetric(const std::string& name,
                   double value,
                   const std::map<std::string, std::string>& tags = {});

    /**
     * @brief Get all collected metrics
     * @return Map of metric names to their values and tags
     */
    std::map<std::string, std::vector<std::pair<double, std::map<std::string, std::string>>>> GetMetrics() const;

    /**
     * @brief Get metrics in Prometheus format
     * @return Metrics as Prometheus text format
     */
    std::string GetMetricsPrometheusFormat() const;

    /**
     * @brief Reset all collected metrics
     */
    void ResetMetrics();

    /**
     * @brief Get aggregated value for a metric
     * @param name Metric name
     * @param aggregation_type Type of aggregation (avg, sum, min, max)
     * @return Aggregated value
     */
    double GetAggregatedValue(const std::string& name, 
                             const std::string& aggregation_type = "avg") const;

private:
    /// Collected metrics: name -> list of (value, tags)
    std::map<std::string, std::vector<std::pair<double, std::map<std::string, std::string>>>> metrics;

    /// Mutex for thread safety
    mutable std::mutex metrics_mutex;
};

/**
 * @brief Tracer implementation
 * Provides distributed tracing capabilities
 */
class Tracer {
public:
    /**
     * @brief Constructor
     */
    Tracer();

    /**
     * @brief Start a new trace span
     * @param operation_name Operation name
     * @param parent_span_id Parent span ID (if any)
     * @return New span ID
     */
    std::string StartSpan(const std::string& operation_name,
                         const std::string& parent_span_id = "");

    /**
     * @brief End a trace span
     * @param span_id Span ID to end
     * @param status Status of the operation
     */
    void EndSpan(const std::string& span_id, const std::string& status = "OK");

    /**
     * @brief Record an event in a span
     * @param span_id Span ID
     * @param name Event name
     * @param attributes Event attributes
     */
    void RecordEvent(const std::string& span_id,
                    const std::string& name,
                    const std::map<std::string, std::string>& attributes = {});

    /**
     * @brief Get trace data
     * @return Vector of trace spans
     */
    std::vector<TraceSpan> GetTraces() const;

private:
    /// Active spans
    std::map<std::string, TraceSpan> active_spans;

    /// Completed spans
    std::vector<TraceSpan> completed_spans;

    /// Mutex for thread safety
    mutable std::mutex trace_mutex;
};

// Inline method implementations
inline bool ObservabilityModule::IsInitialized() const {
    return initialized;
}

inline ObservabilityModule::LogLevel ObservabilityModule::GetLogLevel() const {
    return current_log_level;
}

inline void ObservabilityModule::SetMetricsEnabled(bool enabled) {
    metrics_enabled = enabled;
}

inline void ObservabilityModule::SetTracingEnabled(bool enabled) {
    tracing_enabled = enabled;
}

inline void Logger::Flush() {
    if (output_file.is_open()) {
        output_file.flush();
    }
}

#endif /* OBSERVABILITY_MODULE_H */