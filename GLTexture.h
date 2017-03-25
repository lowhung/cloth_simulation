#pragma once
#include <string>
#include <memory>
#include <nanogui/opengl.h>

class cGLTexture {
public:
	using handleType = std::unique_ptr<uint8_t[], void(*)(void*)>;
	cGLTexture() = default;
	cGLTexture(const std::string& textureName);

	cGLTexture(const std::string& textureName, GLint textureId);

	cGLTexture(const cGLTexture& other) = delete;
	cGLTexture(cGLTexture&& other) noexcept;
	cGLTexture& operator=(const cGLTexture& other) = delete;
	cGLTexture& operator=(cGLTexture&& other) noexcept;
	~cGLTexture() noexcept;

	GLuint texture() const;
	const std::string& textureName() const;

	handleType load(const std::string& fileName);

protected:
	std::string mTextureName;
	GLuint mTextureId;
};