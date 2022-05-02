#include <stdio.h>

#define TEST_FOR_MEMORY_LEAKS 0 // set to 1, on Windows only, to enable memory leak checking on application exit
#if TEST_FOR_MEMORY_LEAKS
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#else
#include <stdlib.h>
#endif

#include <string.h>
#include <stdint.h>

#define ENABLE_VHACD_IMPLEMENTATION 1
#define VHACD_DISABLE_THREADING 0
#include "VHACD.h"
#include "wavefront.h"
#include "FloatMath.h"
#include "ScopedTime.h"

#include <thread>
#include <string>
#include <vector>

#ifdef _MSC_VER
#pragma warning(disable:4100 4996)
#include <conio.h>
#endif

// Evaluates if this is true or false, returns true if it 
// could be evaluated. Stores the result into 'value'
bool getTrueFalse(const char *option,bool &value)
{
	bool ret = false;

	if ( strcmp(option,"t") == 0 ||
		 strcmp(option,"true") == 0 ||
		 strcmp(option,"1") == 0 )
	{
		ret = true;
		value = true;
	}
	else if ( strcmp(option,"f") == 0 ||
		 strcmp(option,"false") == 0 ||
		 strcmp(option,"0") == 0 )
	{
		ret = true;
		value = false;
	}
	else
	{
		printf("Valid values are 'true' or 'false', 't' or 'f', or '1' or '0', only.\n");
	}

	return ret;
}

class Logging : public VHACD::IVHACD::IUserCallback, public VHACD::IVHACD::IUserLogger
{
public:
	Logging(void)
	{
	}

	~Logging(void)
	{
		flushMessages();
	}

        // Be aware that if you are running V-HACD asynchronously (in a background thread) this callback will come from
        // a different thread. So if your print/logging code isn't thread safe, take that into account.
        virtual void Update(const double overallProgress,
                            const double stageProgress,
                            const char* const stage,const char *operation) final
		{
			char scratch[512];
			snprintf(scratch,sizeof(scratch),"[%-40s] : %0.0f%% : %0.0f%% : %s",stage,overallProgress,stageProgress,operation);

			if ( strcmp(stage,mCurrentStage.c_str()) == 0 )
			{
				for (uint32_t i=0; i<mLastLen; i++)
				{
					printf("%c", 8);
				}
			}
			else
			{
				printf("\n");
				mCurrentStage = std::string(stage);
			}
			mLastLen = (uint32_t)strlen(scratch);
			printf("%s", scratch);
		}

        // This is an optional user callback which is only called when running V-HACD asynchronously.
        // This is a callback performed to notify the user that the
        // convex decomposition background process is completed. This call back will occur from
        // a different thread so the user should take that into account.
        virtual void NotifyVHACDComplete(void)
        {
			Log("VHACD::Complete");
        }

		virtual void Log(const char* const msg) final
		{
			mLogMessages.push_back(std::string(msg));
		}

		void flushMessages(void)
		{
			if ( !mLogMessages.empty() )
			{
				printf("\n");
				for (auto &i:mLogMessages)
				{
					printf("%s\n", i.c_str());
				}
				mLogMessages.clear();
			}
		}

		uint32_t	mLastLen{0};
		std::string mCurrentStage;
		std::vector< std::string > mLogMessages;

};

enum class ExportFormat
{
	NONE,
	WAVEFRONT,
	STL
};

const char *lastDot(const char *str)
{
	const char *ret = nullptr;

	while ( *str )
	{
		if ( *str == '.' )
		{
			ret = str;
		}
		str++;
	}

	return ret;
}

int main(int argc,const char **argv)
{
#if TEST_FOR_MEMORY_LEAKS
	_CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
#endif
	if ( argc < 2 )
	{
		printf("Usage: TestVHACD <wavefront.obj> (options)\n");
		printf("\n");
		printf("-h <n>                  : Maximum number of output convex hulls. Default is 32\n");
		printf("-r <voxelresolution>    : Total number of voxels to use. Default is 100,000\n");
		printf("-e <volumeErrorPercent> : Volume error allowed as a percentage. Default is 1%%\n");
		printf("-d <maxRecursionDepth>  : Maximum recursion depth. Default value is 12.\n");
		printf("-s <true/false>         : Whether or not to shrinkwrap output to source mesh. Default is true.\n");
		printf("-f <fillMode>           : Fill mode. Default is 'flood', also 'surface' and 'raycast' are valid.\n");
		printf("-v <maxHullVertCount>   : Maximum number of vertices in the output convex hull. Default value is 64\n");
		printf("-a <true/false>         : Whether or not to run asynchronously. Default is 'true'\n");
		printf("-l <minEdgeLength>      : Minimum size of a voxel edge. Default value is 2 voxels.\n");
		printf("-p <true/false>         : If false, splits hulls in the middle. If true, tries to find optimal split plane location. False by default.\n");
		printf("-o <obj/stl>            : Export the convex hulls as a series of wavefront OBJ files or STL files.\n");
	}
	else
	{
		Logging logging;
		VHACD::IVHACD::Parameters p;
		p.m_callback = &logging;
		p.m_logger = &logging;
		const char *inputFile = argv[1];

		ExportFormat format = ExportFormat::NONE;

		WavefrontObj w;
		uint32_t tcount = w.loadObj(inputFile);
		if ( tcount == 0 )
		{
			printf("Failed to load valid mesh from wavefront OBJ file:%s\n", inputFile);
		}
		else
		{
			for (int i=2; i<argc; i+=2)
			{
				const char *option = argv[i];
				const char *value = argv[i+1];
				if ( strcmp(option,"-h") == 0 )
				{
					int32_t v = atoi(value);
					if ( v < 1 || v > 100000 )
					{
						printf("Invalid maximum number of convex hulls. Valid range is 1-100000\n");
					}
					else
					{
						p.m_maxConvexHulls = uint32_t(v);
						printf("Maximum convex hulls set to:%d\n", p.m_maxConvexHulls);
					}
				}
				else if ( strcmp(option,"-r") == 0 )
				{
					int32_t r = atoi(value);
					if ( r >= 10000 && r <= 10000000 )
					{
						printf("Voxel Resolution set to:%d\n", r);
						p.m_resolution = uint32_t(r);
					}
					else
					{
						printf("Invalid voxel resolution must be between 10,000 and 10,000,000\n");
					}
				}
				else if ( strcmp(option,"-e") == 0 )
				{
					double e = atof(value);
					if ( e < 0 || e > 50 )
					{
						printf("Invalid error percentage. Valid values are 0 to 50\n");
					}
					else
					{
						p.m_minimumVolumePercentErrorAllowed = e;
						printf("Minimum volume error allowed set to:%0.2f%%\n", p.m_minimumVolumePercentErrorAllowed);
					}
				}
				else if ( strcmp(option,"-o") == 0 )
				{
					if ( strcmp(value,"obj") == 0 )
					{
						format = ExportFormat::WAVEFRONT;
						printf("Saving the output convex hulls as a series of wavefront OBJ files.\n");
					}
					else if ( strcmp(value,"stl") == 0 )
					{
						format = ExportFormat::STL;
						printf("Saving the output convex hulls as a single ASCII STL file.\n");
					}
					else
					{
						printf("Unknown export file format (%s). Currently only support 'obj' and 'stl'\n", value);
					}
				}
				else if ( strcmp(option,"-d") == 0 )
				{
					int32_t r = atoi(value);
					if ( r >= 2 && r <= 64 )
					{
						printf("Maximum recursion depth set to:%d\n", r);
						p.m_maxRecursionDepth = uint32_t(r);
					}
					else
					{
						printf("Invalid maximum recursion depth, must be between 2 and 64\n");
					}
				}
				else if ( strcmp(option,"-s") == 0 )
				{
					if ( getTrueFalse(value,p.m_shrinkWrap) )
					{
						if ( p.m_shrinkWrap )
						{
							printf("Shrinkwrap enabled.\n");
						}
						else
						{
							printf("Shrinkwrap disabled.\n");
						}
					}
				}
				else if ( strcmp(option,"-f") == 0 )
				{
					if ( strcmp(value,"flood") == 0 )
					{
						p.m_fillMode = VHACD::FillMode::FLOOD_FILL;
						printf("FillMode set to FLOOD\n");
					}
					else if ( strcmp(value,"raycast") == 0 )
					{
						p.m_fillMode = VHACD::FillMode::RAYCAST_FILL;
						printf("FillMode set to RAYCAST\n");
					}
					else if ( strcmp(value,"surface") == 0 )
					{
						p.m_fillMode = VHACD::FillMode::SURFACE_ONLY;
						printf("FillMode set to SURFACE\n");
					}
					else
					{
						printf("Invalid fill mode, only valid options are 'flood', 'raycast', and 'surface'\n");
					}
				}
				else if ( strcmp(option,"-v") == 0 )
				{
					int32_t r = atoi(value);
					if ( r >= 8 && r <= 2048 )
					{
						printf("Maximum hull vertices set to:%d\n", r);
						p.m_maxNumVerticesPerCH = uint32_t(r);
					}
					else
					{
						printf("Invalid maximum hull vertices, must be between 8 and 20484\n");
					}
				}
				else if ( strcmp(option,"-a") == 0 )
				{
					if ( getTrueFalse(value,p.m_asyncACD) )
					{
						if ( p.m_asyncACD )
						{
							printf("Asynchronous mode enabled\n");
						}
						else
						{
							printf("Synchronous mode disabled\n");
						}
					}
				}
				else if ( strcmp(option,"-p") == 0 )
				{
					if ( getTrueFalse(value,p.m_findBestPlane) )
					{
						if ( p.m_asyncACD )
						{
							printf("Find best split plane location enabled\n");
						}
						else
						{
							printf("Using binary plane split\n");
						}
					}
				}
				else if ( strcmp(option,"-l") == 0 )
				{
					int32_t r = atoi(value);
					if ( r >= 1 && r <= 32 )
					{
						printf("Minimum voxel edge length set to:%d\n", r);
						p.m_minEdgeLength = uint32_t(r);
					}
					else
					{
						printf("Invalid minimum voxel edge length, must be between 1 and 32\n");
					}
				}
			}
#if VHACD_DISABLE_THREADING
			VHACD::IVHACD *iface = VHACD::CreateVHACD();
#else
			VHACD::IVHACD *iface = VHACD::CreateVHACD_ASYNC();
#endif
#ifdef _MSC_VER
			printf("Press the SPACEBAR to cancel convex decomposition before it has completed.\n");
#endif
			double *points = new double[w.mVertexCount*3];
			for (uint32_t i=0; i<w.mVertexCount*3; i++)
			{
				points[i] = w.mVertices[i];
			}
			bool canceled = false;
			{
				ScopedTime st("Computing Convex Decomposition");
				iface->Compute(points,w.mVertexCount,w.mIndices,w.mTriCount,p);
				while ( !iface->IsReady() )
				{
					std::this_thread::sleep_for(std::chrono::nanoseconds(10000)); // s
#ifdef _MSC_VER
					if ( kbhit() )
					{
						char c = (char)getch();
						if ( c == 32 )
						{
							printf("Canceling convex decomposition.\n");
							iface->Cancel();
							canceled = true;
						}
					}
#endif
				}
				logging.flushMessages();
			}
			if ( !canceled && iface->GetNConvexHulls() )
			{
				const char *fname = argv[1];
				const char *dot = lastDot(fname);
				if ( dot && format != ExportFormat::NONE)
				{
					std::string baseName;
					while ( fname != dot )
					{
						baseName.push_back(*fname);
						fname++;
					}
					if ( format == ExportFormat::STL )
					{
						char outputName[2048];
						snprintf(outputName,sizeof(outputName),"%s_decompose.stl", baseName.c_str());
						FILE *fph = fopen(outputName,"wb");
						if ( fph )
						{
							printf("Saving:%s\n", outputName);
							for (uint32_t i=0; i<iface->GetNConvexHulls(); i++)
							{
								VHACD::IVHACD::ConvexHull ch;
								iface->GetConvexHull(i,ch);
								uint32_t baseIndex = 1;
								if ( fph )
								{
									char hullName[2048];
									snprintf(hullName,sizeof(hullName),"%s%03d.obj", baseName.c_str(), i);
									fprintf(fph,"solid %s\n", hullName);
									for (uint32_t j=0; j<ch.m_nTriangles; j++)
									{
										uint32_t i1 = ch.m_triangles[j*3+0];
										uint32_t i2 = ch.m_triangles[j*3+1];
										uint32_t i3 = ch.m_triangles[j*3+2];

										const double *p1 = &ch.m_points[i1*3];
										const double *p2 = &ch.m_points[i2*3];
										const double *p3 = &ch.m_points[i3*3];

										double normal[3];
										FLOAT_MATH::fm_computePlane(p1,p2,p3,normal);
										fprintf(fph," facet normal %0.9f %0.9f %0.9f\n", normal[0], normal[1], normal[2]);
										fprintf(fph,"  outer loop\n");
										fprintf(fph,"   vertex %0.9f %0.9f %0.9f\n", p1[0], p1[1], p1[2]);
										fprintf(fph,"   vertex %0.9f %0.9f %0.9f\n", p2[0], p2[1], p2[2]);
										fprintf(fph,"   vertex %0.9f %0.9f %0.9f\n", p3[0], p3[1], p3[2]);
										fprintf(fph,"  endloop\n");
										fprintf(fph," endfacet\n");
									}
									fprintf(fph,"endsolid %s\n", hullName);
								}
							}
						}
						else
						{
							printf("Failed to open output file (%s) for write access.\n", outputName);
						}
					}
					else
					{
						for (uint32_t i=0; i<iface->GetNConvexHulls(); i++)
						{
							VHACD::IVHACD::ConvexHull ch;
							iface->GetConvexHull(i,ch);
							char outputName[2048];
							snprintf(outputName,sizeof(outputName),"%s%03d.obj", baseName.c_str(), i);
							FILE *fph = fopen(outputName,"wb");
							uint32_t baseIndex = 1;
							if ( fph )
							{
								printf("Saving:%s\n", outputName);
								for (uint32_t j=0; j<ch.m_nPoints; j++)
								{
									const double *pos = &ch.m_points[j*3];
									fprintf(fph,"v %0.9f %0.9f %0.9f\n", pos[0], pos[1], pos[2]);
								}
								for (uint32_t j=0; j<ch.m_nTriangles; j++)
								{
									uint32_t i1 = ch.m_triangles[j*3+0]+baseIndex;
									uint32_t i2 = ch.m_triangles[j*3+1]+baseIndex;
									uint32_t i3 = ch.m_triangles[j*3+2]+baseIndex;
									fprintf(fph,"f %d %d %d\n", i1, i2, i3);
								}
								fclose(fph);
							}
							else
							{
								printf("Failed to open output file '%s' for write access\n", outputName);
							}
						}
					}
				}
				FILE *fph = fopen("decomp.obj", "wb");
				if ( fph )
				{
					printf("Saving Convex Decomposition results of %d convex hulls to 'decomp.obj'\n", iface->GetNConvexHulls());
					uint32_t baseIndex = 1;
					for (uint32_t i=0; i<iface->GetNConvexHulls(); i++)
					{
						VHACD::IVHACD::ConvexHull ch;
						iface->GetConvexHull(i,ch);
						for (uint32_t j=0; j<ch.m_nPoints; j++)
						{
							const double *pos = &ch.m_points[j*3];
							fprintf(fph,"v %0.9f %0.9f %0.9f\n", pos[0], pos[1], pos[2]);
						}
						for (uint32_t j=0; j<ch.m_nTriangles; j++)
						{
							uint32_t i1 = ch.m_triangles[j*3+0]+baseIndex;
							uint32_t i2 = ch.m_triangles[j*3+1]+baseIndex;
							uint32_t i3 = ch.m_triangles[j*3+2]+baseIndex;
							fprintf(fph,"f %d %d %d\n", i1, i2, i3);
						}
						baseIndex+=ch.m_nPoints;
					}
					fclose(fph);
				}
			}

			delete []points;
			iface->Release();


		}
	}
	return 0;
}
