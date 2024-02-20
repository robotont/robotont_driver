#ifndef ROBOTONT_DRIVER_EXCEPTION_HPP
#define ROBOTONT_DRIVER_EXCEPTION_HPP

// DriverException struct that inherits from std::exception
struct DriverException : public std::exception
{
  // Description of the exception
  std::string desc_;
  // Constructor that initializes the description
  DriverException(std::string desc) : desc_(desc)
  {
  }

  // Destructor
  ~DriverException() throw()
  {
  }

  // Function to return the description of the exception
  const char* what() const throw()
  {
    return desc_.c_str();
  }
};

#endif