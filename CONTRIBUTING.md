# Contribution Guidelines 🚀

Thank you for your interest in contributing to the **ROS2 Arctos Repository**! 🎉 We welcome contributions from the community to help improve and expand the functionality of the Arctos 6DOF robotic arm. Please review these guidelines to ensure a smooth collaboration process. 🤝

## How to Contribute 🛠️

### 1. Reporting Issues 🐛

If you encounter a bug, have a feature request, or notice something that can be improved, please open an issue in the repository. Ensure that your issue includes:

- 📝 A clear and descriptive title.
- 🔄 Steps to reproduce the problem (if reporting a bug).
- ✅ Expected behavior and 🚫 actual behavior.
- 📸 Relevant logs, screenshots, or error messages.

### 2. Submitting Code Contributions 💻

We appreciate code contributions in the form of pull requests. Follow these steps:

1. **Fork the Repository**: 🍴 Create a fork of the repository on GitHub.
2. **Create a Branch**: 🌿 Create a new branch for your feature or bug fix:
3. 
   ```bash
   git checkout -b feature/your-feature-name
   ```
   
4. **Write Code**: ✍️ Implement your changes, following the coding standards and best practices outlined below.
5. **Test Your Changes**: ✅ Ensure your changes are properly tested and do not break existing functionality.
6. **Commit Changes**: 💾 Write clear and concise commit messages:

   ```bash
   git commit -m "Add feature XYZ"
   ```
   
8. **Push to Your Fork**: 🚀 Push your changes to your forked repository:

   ```bash
   git push origin feature/your-feature-name
   ```
   
10. **Submit a Pull Request**: 🔀 Open a pull request against the `main` branch of the original repository.

### 3. Reviewing Pull Requests 👀

All pull requests will be reviewed by maintainers or contributors with write access. During the review process:

- 🔄 Be responsive to feedback and make requested changes promptly.
- 💡 Discuss potential improvements or alternative solutions as needed.

### 4. Adding Documentation 📚

Improving the documentation is as valuable as writing code. You can contribute by:

- ✍️ Updating or creating new markdown files.
- 📝 Enhancing inline code comments.
- 🌟 Adding examples or tutorials to help new users get started.

## Coding Standards 📏

To ensure consistency and maintainability, please follow these coding guidelines:

- **Python**: 🐍 Use Python 3.x and adhere to [PEP 8](https://peps.python.org/pep-0008/) guidelines.
- **C++**: ⚙️ Follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
- **ROS2 Developer Guide**: Refer to the [ROS2 Developer Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html#).
- **Code Style and Language**: Follow the [ROS2 Code Style and Language guidelines](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
- **Documentation**: 🖋️ Include docstrings for functions and classes.
- **Testing**: 🧪 Write unit tests for new features or changes, using the [ROS2 testing framework](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html#).

## Branching Model 🌳

We follow a simple branching strategy:

- **`main`**: 🌟 The stable branch containing production-ready code.
- **`humble`**: 🤖 The branch specific to ROS 2 Humble, ensuring compatibility with this distribution.
- **`develop`**: 🚧 The branch for ongoing development. All pull requests should target this branch.
- **Feature branches**: 🛠️ Create a branch from `develop` for new features or bug fixes, e.g., `feature/xyz` or `bugfix/abc`.

## Code of Conduct 🤝

This project adheres to the [Code of Conduct](CODE_OF_CONDUCT.md). By participating, you are expected to uphold this standard.

## Getting Help 💬

If you need help with your contributions or have questions about the project, feel free to:

- 💡 Join the discussion on the [Issues](https://github.com/Arctos-Robotics/ros2_arctos/issues) page.
- 🛠️ Reach out to maintainers by tagging them in your pull request or issue.

---

We’re excited to have you contribute to the ROS2 Arctos project! 🎉 Thank you for helping us make this project better for everyone. 🌍
