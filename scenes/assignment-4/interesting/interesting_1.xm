<?xml version='1.0' encoding='utf-8'?>

<scene>
    <integrator type="path_mis"/>


    <camera type="perspective">
        <!-- 3D origin, target point, and 'up' vector -->
		<transform name="toWorld">
			<lookat target="0, 0, -1" 
                    origin="1, 3.5, 5.5" up="0, 1, 0"
            />
		</transform>

		<!-- Field of view: 45 degrees -->
		<float name="fov" value="30"/>

		<!-- 768 x 768 pixels -->
		<integer name="height" value="720"/>
        <integer name="width" value="1280"/>
	</camera>

    <sampler type="independent">
        <integer name="sampleCount" value="4096"/>
    </sampler>

    <!-- Checkerboard Floor -->
    <mesh type="obj">
        <string name="filename" value="meshes/plane.obj"/>
        <bsdf type="diffuse">
            <texture name="albedo" type="checker">
                <integer name="scalex" value="100"/>
			    <integer name="scaley" value="100"/>
            </texture>
        </bsdf>
    </mesh>

    <mesh type="obj">
    <!-- back wall -->
		<string name="filename" value="meshes/rightwall.obj"/>
        <transform name="toWorld">
            <rotate axis="0,1,0" angle="90" />
            <scale value="1,0.7,1"/>
        </transform>
		<bsdf type="diffuse">
            <color name="albedo" value="0.065 0.63 0.05"/>
		</bsdf>
	</mesh>

    <mesh type="obj">
		<string name="filename" value="meshes/rightwall.obj"/>
        <transform name="toWorld">
            <scale value="1,0.7,1"/>
        </transform>
		<bsdf type="diffuse">
			<color name="albedo" value="0.161 0.133 0.427"/>
		</bsdf>
	</mesh>

    <mesh type="obj">
		<string name="filename" value="meshes/leftwall.obj"/>
        <transform name="toWorld">
            <scale value="1,0.7,1"/>
        </transform>
		<bsdf type="diffuse">
			<color name="albedo" value="0.630 0.065 0.05"/>
		</bsdf>
	</mesh>
    
    <mesh type="obj">
        <string name="filename" value="meshes/king.obj"/>
        <transform name="toWorld">
            <translate value="-1,16,0"/>
            <rotate axis="1,0,0" angle="-120"/>
            <rotate axis="0,1,0" angle="-90"/>
            <scale value="0.08,0.08,0.08"/>
        </transform>
        
        <bsdf type="roughconductor">
			<color name="R0" value="0.91 0.92 0.92"/>
			<float name="alpha" value="0.3"/>
		</bsdf>
    </mesh>
    
    <mesh type="obj">
        <string name="filename" value="meshes/queen.obj"/>
        <transform name="toWorld">
            <translate value="-3,16,0"/>
            <rotate axis="1,0,0" angle="-90"/>
            <scale value="0.05,0.05,0.05"/>
        </transform>
        <bsdf type="roughconductor">
			<color name="R0" value="1.0 0.71 0.29"/>
			<float name="alpha" value="0.1"/>
		</bsdf>
    </mesh>

    <!-- Glass Sphere -->
    <mesh type="obj">
        <string name="filename" value="meshes/sphere1.obj"/>
        <transform name="toWorld">
            <translate value="-0.05,0.45,1.5"/>
            <scale value="0.5,0.5,0.5"/>
        </transform>
        <bsdf type="dielectric">
            <float name="intIOR" value="1.5"/>
            <float name="extIOR" value="1.0"/>
        </bsdf>
    </mesh>

    <!-- Mirror Sphere -->
    <mesh type="obj">
        <string name="filename" value="meshes/sphere2.obj"/>
        <bsdf type="mirror"/>
    </mesh>

    <!-- Area Light -->
    <mesh type="obj">
        <string name="filename" value="meshes/light.obj"/>
        <transform name="toWorld">
            <scale value="2.5,2.5,2.5"/>
        </transform>
        <emitter type="area">
            <color name="radiance" value="50, 50, 50"/>
        </emitter>
    </mesh>
</scene>
