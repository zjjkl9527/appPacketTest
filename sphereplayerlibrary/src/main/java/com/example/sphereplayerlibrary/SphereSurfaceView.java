package com.example.sphereplayerlibrary;

import android.content.Context;
import android.graphics.SurfaceTexture;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.util.AttributeSet;
import android.view.Surface;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import static android.opengl.GLES11Ext.GL_TEXTURE_EXTERNAL_OES;

public class SphereSurfaceView extends GLSurfaceView  {

    private final SphereRenderer mRenderer=new SphereRenderer();
    private final float[] mRotationMatrix = new float[16];

    public SphereSurfaceView(Context context) {
        super(context);
        init(this,context);
    }

    public SphereSurfaceView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    private void init(SphereSurfaceView sphereSurfaceView, Context context){
        sphereSurfaceView.setEGLContextClientVersion(3);
        sphereSurfaceView.setRenderer(mRenderer);
        sphereSurfaceView.setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
        Matrix.setIdentityM(mRotationMatrix, 0);
    }

    public SphereRenderer getSphereRender(){
        return mRenderer;
    }


    static class SphereRenderer implements Renderer {
        private Surface mSurface;
        private SurfaceTexture mTexture;
        private int mProgram;
        private int mPositionHandle;
        private int mTexCoordHandle;
        private int mMatrixHandle;
        private int mSamplerHandle;
        private FloatBuffer mVertexBuffer;
        private FloatBuffer mTexBuffer;
        private final int mTriangleNum;

        private final float[] mProjectionMatrix = new float[16];
        private final float[] mCameraMatrix = new float[16];

        private float mAspect = 2.0f;
        private float mFov = 60.f;
        public float mRotateX = 0.f;
        public float mRotateY = 0.f;

        static final private  String VERTEX_SHADER = "uniform mat4 uMVPMatrix;" +
                "attribute vec4 vPosition;" +
                "attribute vec2 a_texCoord;" +
                "varying vec2 v_texCoord;" +
                "void main() {" +
                "  gl_Position = uMVPMatrix * vPosition;" +
                "  v_texCoord = a_texCoord;" +
                "}";

        static final private  String FRAGMENT_SHADER = "#extension GL_OES_EGL_image_external : require\n" +
                "precision mediump float;" +
                "varying vec2 v_texCoord;" +
                "uniform samplerExternalOES s_texture;" +
                "void main() {" +
                "  gl_FragColor = texture2D( s_texture, v_texCoord );" +
                "}";


        public SphereRenderer(){
            mTriangleNum = generateGeometry();
            Matrix.setLookAtM(mCameraMatrix, 0, 0, 0, 0, 0, 0,
                    1000, 0, 1, 0);
        }

        public Surface getSurface(){
            if(mSurface!= null) return mSurface;

            int[] textureID = new int[1];
            GLES20.glGenTextures(1, textureID, 0);
            int externalTextureId = textureID[0];

            GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
            GLES20.glBindTexture(
                    GL_TEXTURE_EXTERNAL_OES,
                    externalTextureId);
            GLES20.glTexParameterf(
                    GL_TEXTURE_EXTERNAL_OES,
                    GLES20.GL_TEXTURE_MIN_FILTER,
                    GLES20.GL_LINEAR);
            GLES20.glTexParameterf(
                    GL_TEXTURE_EXTERNAL_OES,
                    GLES20.GL_TEXTURE_MAG_FILTER,
                    GLES20.GL_LINEAR);
            GLES20.glTexParameteri(
                    GL_TEXTURE_EXTERNAL_OES,
                    GLES20.GL_TEXTURE_WRAP_S,
                    GLES20.GL_CLAMP_TO_EDGE);
            GLES20.glTexParameteri(
                    GL_TEXTURE_EXTERNAL_OES,
                    GLES20.GL_TEXTURE_WRAP_T,
                    GLES20.GL_CLAMP_TO_EDGE);


            mTexture = new SurfaceTexture(externalTextureId);
            mTexture.setDefaultBufferSize(100, 100);


            mSurface = new Surface(mTexture);
            return mSurface;
        }

        public void zoom(float val){
            mFov += val;

            if(mFov > 178.f) mFov = 178.f;
            if(mFov < 1.f) mFov = 1.f;
        }


        public void rotateX(float val){
            mRotateX += val;
        }

        public void rotateY(float val){
            mRotateY += val;
        }

        public void setXRotation(float val){
            mRotateX = val;
        }

        public void setYRotation(float val){
            mRotateY = val;
        }



        @Override
        public void onSurfaceCreated(GL10 gl, EGLConfig config) {
            getSurface();
        }

        @Override
        public void onSurfaceChanged(GL10 gl, int width, int height) {
            mProgram = GLES20.glCreateProgram();

            int vertexShader = GLES20.glCreateShader(GLES20.GL_VERTEX_SHADER);
            GLES20.glShaderSource(vertexShader, VERTEX_SHADER);
            GLES20.glCompileShader(vertexShader);

            int fragmentShader = GLES20.glCreateShader(GLES20.GL_FRAGMENT_SHADER);
            GLES20.glShaderSource(fragmentShader, FRAGMENT_SHADER);
            GLES20.glCompileShader(fragmentShader);

            GLES20.glAttachShader(mProgram, vertexShader);
            GLES20.glAttachShader(mProgram, fragmentShader);

            GLES20.glLinkProgram(mProgram);

            mPositionHandle = GLES20.glGetAttribLocation(mProgram, "vPosition");
            mTexCoordHandle = GLES20.glGetAttribLocation(mProgram, "a_texCoord");
            mMatrixHandle = GLES20.glGetUniformLocation(mProgram, "uMVPMatrix");
            mSamplerHandle = GLES20.glGetUniformLocation(mProgram, "s_texture");

            mAspect = (float) width / height;
            Matrix.perspectiveM(mProjectionMatrix, 0, mFov,  mAspect,
                    1, 1000f);
        }

        @Override
        public void onDrawFrame(GL10 gl) {
            mTexture.updateTexImage();

            Matrix.perspectiveM(mProjectionMatrix, 0, mFov,  mAspect,
                    1, 1000f);
            float[] mvpMatrix = new float[16];
            Matrix.multiplyMM(mvpMatrix, 0, mProjectionMatrix, 0, mCameraMatrix, 0);
            Matrix.rotateM(mvpMatrix, 0, mRotateX, 1, 0, 0);
            Matrix.rotateM(mvpMatrix, 0, mRotateY, 0, 1, 0);

            GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);
            GLES20.glUseProgram(mProgram);
            GLES20.glEnableVertexAttribArray(mPositionHandle);
            GLES20.glVertexAttribPointer(mPositionHandle, 3, GLES20.GL_FLOAT, false,
                    12, mVertexBuffer);
            GLES20.glEnableVertexAttribArray(mTexCoordHandle);
            GLES20.glVertexAttribPointer(mTexCoordHandle, 2, GLES20.GL_FLOAT, false, 0,
                    mTexBuffer);
            GLES20.glUniformMatrix4fv(mMatrixHandle, 1, false, mvpMatrix, 0);
            GLES20.glUniform1i(mSamplerHandle, 0);

            GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, mTriangleNum);
            GLES20.glDisableVertexAttribArray(mPositionHandle);
        }

        private int generateGeometry(){
            final int CAP = 9;
            final float r = 6;
            final int triangleNum = (180 / CAP) * (360 / CAP) * 6 ;
            float[] vertices = new float[triangleNum* 3];
            float[] tex = new float[triangleNum * 2];

            float x = 0;
            float y = 0;
            float z = 0;
            int index = 0;
            int index1 = 0;
            double d = CAP * Math.PI / 180;
            for (int i = 0; i < 180; i += CAP) {
                double d1 = i * Math.PI / 180;
            for (int j = 0; j < 360; j += CAP) {
                double d2 = j * Math.PI / 180;
                    vertices[index++] = (float) (x + r * Math.sin(d1 + d) * Math.cos(d2 + d));
                    vertices[index++] = (float) (y + r * Math.cos(d1 + d));
                    vertices[index++] = (float) (z + r * Math.sin(d1 + d) * Math.sin(d2 + d));
                    tex[index1++] = (j + CAP) * 1f / 360;
                    tex[index1++] = (i + CAP) * 1f / 180;

                    vertices[index++] = (float) (x + r * Math.sin(d1) * Math.cos(d2));
                    vertices[index++] = (float) (y + r * Math.cos(d1));
                    vertices[index++] = (float) (z + r * Math.sin(d1) * Math.sin(d2));

                    tex[index1++] = j * 1f / 360;
                    tex[index1++] = i * 1f / 180;

                    vertices[index++] = (float) (x + r * Math.sin(d1) * Math.cos(d2 + d));
                    vertices[index++] = (float) (y + r * Math.cos(d1));
                    vertices[index++] = (float) (z + r * Math.sin(d1) * Math.sin(d2 + d));

                    tex[index1++] = (j + CAP) * 1f / 360;
                    tex[index1++] = i * 1f / 180;

                    vertices[index++] = (float) (x + r * Math.sin(d1 + d) * Math.cos(d2 + d));
                    vertices[index++] = (float) (y + r * Math.cos(d1 + d));
                    vertices[index++] = (float) (z + r * Math.sin(d1 + d) * Math.sin(d2 + d));

                    tex[index1++] = (j + CAP) * 1f / 360;
                    tex[index1++] = (i + CAP) * 1f / 180;

                    vertices[index++] = (float) (x + r * Math.sin(d1 + d) * Math.cos(d2));
                    vertices[index++] = (float) (y + r * Math.cos(d1 + d));
                    vertices[index++] = (float) (z + r * Math.sin(d1 + d) * Math.sin(d2));

                    tex[index1++] = j * 1f / 360;
                    tex[index1++] = (i + CAP) * 1f / 180;

                    vertices[index++] = (float) (x + r * Math.sin(d1) * Math.cos(d2));
                    vertices[index++] = (float) (y + r * Math.cos(d1));
                    vertices[index++] = (float) (z + r * Math.sin(d1) * Math.sin(d2));

                    tex[index1++] = j * 1f / 360;
                    tex[index1++] = i * 1f / 180;
                }
            }
            mVertexBuffer = ByteBuffer.allocateDirect(vertices.length * 4)
                    .order(ByteOrder.nativeOrder())
                    .asFloatBuffer()
                    .put(vertices);
            mVertexBuffer.position(0);

            mTexBuffer = ByteBuffer.allocateDirect(tex.length * 4)
                    .order(ByteOrder.nativeOrder())
                    .asFloatBuffer()
                    .put(tex);
            mTexBuffer.position(0);

            return triangleNum;
        }


    }

}



