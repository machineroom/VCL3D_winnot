#include "viewer.h"
#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


Viewer::Viewer() : shader_folder("src/shader/"), 
                   win_width(800),
                   win_height(400)
{
}

static void glfwErrorCallback(int error, const char* description)
{
  std::cerr << "GLFW error " << error << " " << description << std::endl;
}

void Viewer::initialize()
{
    // init glfw - if already initialized nothing happens
    glfwInit();

    GLFWerrorfun prev_func = glfwSetErrorCallback(glfwErrorCallback);
    if (prev_func)
      glfwSetErrorCallback(prev_func);

    // setup context
    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
#ifdef __APPLE__
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#else
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_ANY_PROFILE);
#endif
    //glfwWindowHint(GLFW_VISIBLE, debug ? GL_TRUE : GL_FALSE);

    window = glfwCreateWindow(win_width*2, win_height*2, "Viewer (press ESC to exit)", 0, NULL);
    if (window == NULL)
    {
        std::cerr << "Failed to create opengl window." << std::endl;
        exit(-1);
    }

    glfwMakeContextCurrent(window);
    OpenGLBindings *b = new OpenGLBindings();
    flextInit(b);
    gl(b);

    std::string vertexshadersrc = ""
        "#version 330\n"
                                                
        "in vec2 Position;"
        "in vec2 TexCoord;"
                    
        "out VertexData{"
        "vec2 TexCoord;" 
        "} VertexOut;"  
                    
        "void main(void)"
        "{"
        "    gl_Position = vec4(Position, 0.0, 1.0);"
        "    VertexOut.TexCoord = TexCoord;"
        "}";
    std::string grayfragmentshader = ""
        "#version 330\n"
        
        "uniform sampler2DRect Data;"
        
        "vec4 tempColor;"
        "in VertexData{"
        "    vec2 TexCoord;"
        "} FragmentIn;"
        
        "layout(location = 0) out vec4 Color;"
        
        "void main(void)"
        "{"
            "ivec2 uv = ivec2(FragmentIn.TexCoord.x, FragmentIn.TexCoord.y);"
            "tempColor = texelFetch(Data, uv);"
            "Color = vec4(tempColor.x/4500, tempColor.x/4500, tempColor.x/4500, 1);"
        "}";
    std::string fragmentshader = ""
        "#version 330\n"
        
        "uniform sampler2DRect Data;"
        
        "in VertexData{"
        "    vec2 TexCoord;"
        "} FragmentIn;"
       
        "layout(location = 0) out vec4 Color;"
        
        "void main(void)"
        "{"
        "    ivec2 uv = ivec2(FragmentIn.TexCoord.x, FragmentIn.TexCoord.y);"

        "    Color = texelFetch(Data, uv);"
        "}";

    renderShader.setVertexShader(vertexshadersrc);
    renderShader.setFragmentShader(fragmentshader);
    renderShader.build();

    renderGrayShader.setVertexShader(vertexshadersrc);
    renderGrayShader.setFragmentShader(grayfragmentshader);
    renderGrayShader.build();


    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, Viewer::key_callbackstatic);
    glfwSetWindowSizeCallback(window, Viewer::winsize_callbackstatic);

    shouldStop = false;
}

void Viewer::winsize_callbackstatic(GLFWwindow* window, int w, int h)
{
    Viewer* viewer = reinterpret_cast<Viewer*>(glfwGetWindowUserPointer(window));
    viewer->winsize_callback(window, w, h);
}

void Viewer::winsize_callback(GLFWwindow* window, int w, int h)
{
    win_width = w/2;
    win_height = h/2;
}

void Viewer::key_callbackstatic(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    Viewer* viewer = reinterpret_cast<Viewer*>(glfwGetWindowUserPointer(window));
    viewer->key_callback(window, key, scancode, action, mods);
}

void Viewer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        shouldStop = true;
}

void Viewer::onOpenGLBindingsChanged(OpenGLBindings *b)
{
    renderShader.gl(b);
    renderGrayShader.gl(b);
    rgb.gl(b);
    ir.gl(b);
}

// start a new render, maybe 1 or more frames rendered after this
void Viewer::start()
{
    // wipe the drawing surface clear
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

//botom,right control placement in 2*2 grid
bool Viewer::render_colour(uint8_t *frame_data, int frame_width, int frame_height, int frame_bytes_per_pixel, Position pos, std::string title)
{

    int fb_width, fb_height;

    // Using the frame buffer size to account for screens where window.size != framebuffer.size, e.g. retina displays
    glfwGetFramebufferSize(window, &fb_width, &fb_height);
    int fb_width_third = (fb_width + 1) / 3;
	int fb_height_half = (fb_height + 1) / 2;

	if (title != "") {
		//title
		cv::Mat mat (frame_height,frame_width,CV_8UC4,frame_data);
	   	//cv::putText (InputOutputArray img, const String &text, Point org, int fontFace, double fontScale, Scalar color, int thickness=1, int lineType=LINE_8, bool bottomLeftOrigin=false)
	   	int scale=1;
	   	int thickness=1;
	   	if (frame_height > 500) {
	   		//help title show up on small buffers
	   		scale = 3;
	   		thickness=2;
	   	}
	   	cv::putText(mat, title, cv::Point(0,50), cv::FONT_HERSHEY_DUPLEX, scale, cv::Scalar(255,255,255), thickness, cv::LINE_AA);
	}
	int x,y;
	switch (pos) {
		case TOP_LEFT:
			x=0; y=fb_height_half;
			break;
		case TOP_MIDDLE:
			x=fb_width_third; y=fb_height_half;
			break;
		case TOP_RIGHT:
			x=fb_width_third*2; y=fb_height_half;
			break;
		case BOTTOM_LEFT:
			x=0; y=0;
			break;
		case BOTTOM_MIDDLE:
			x=fb_width_third; y=0;
			break;
		case BOTTOM_RIGHT:
			x=fb_width_third*2; y=0;
			break;
	}
    glViewport(x, y, fb_width_third, fb_height_half);

    float w = static_cast<float>(frame_width);
    float h = static_cast<float>(frame_height);

    Vertex bl = { -1.0f, -1.0f, 0.0f, 0.0f };
    Vertex br = { 1.0f, -1.0f, w, 0.0f }; 
    Vertex tl = { -1.0f, 1.0f, 0.0f, h };
    Vertex tr = { 1.0f, 1.0f, w, h };
    Vertex vertices[] = {
        bl, tl, tr, 
        tr, br, bl
    };

    gl()->glGenBuffers(1, &triangle_vbo);
    gl()->glGenVertexArrays(1, &triangle_vao);

    gl()->glBindVertexArray(triangle_vao);
    gl()->glBindBuffer(GL_ARRAY_BUFFER, triangle_vbo);
    gl()->glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    GLint position_attr = renderShader.getAttributeLocation("Position");
    gl()->glVertexAttribPointer(position_attr, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
    gl()->glEnableVertexAttribArray(position_attr);

    GLint texcoord_attr = renderShader.getAttributeLocation("TexCoord");
    gl()->glVertexAttribPointer(texcoord_attr, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)(2 * sizeof(float)));
    gl()->glEnableVertexAttribArray(texcoord_attr);


    renderShader.use();

    rgb.allocate(frame_width, frame_height);
    std::copy(frame_data, frame_data + frame_width * frame_height * frame_bytes_per_pixel, rgb.data);
    rgb.flipY();
    rgb.upload();
    glDrawArrays(GL_TRIANGLES, 0, 6);

    rgb.deallocate();

    gl()->glDeleteBuffers(1, &triangle_vbo);
    gl()->glDeleteVertexArrays(1, &triangle_vao);
    return true;
}

// return true if should quit
bool Viewer::finish() {
    // put the stuff we've been drawing onto the display
    glfwSwapBuffers(window);
    // update other events like input handling 
    glfwPollEvents();
    return shouldStop || glfwWindowShouldClose(window);
}


