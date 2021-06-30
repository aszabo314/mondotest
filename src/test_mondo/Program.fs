open System
open Aardvark.Base
open Aardvark.Rendering
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Slim
open FSharp.Data.Adaptive

open Aardvark.Reconstruction
open Mondo.Api

[<EntryPoint;STAThread>]
let main argv = 
    Aardvark.Init()
    
    let proj =         
        {
            //ratio of the shot's focal length to 35mm equivalent
            focalLength = 1.3691585315841093

            //width/height
            aspect = 1.4994888052571265

            distortion = Distortion.empty
        }

    let cfg = 
        { PhotogrammetryConfig.Default proj with
            //uniqueness of features = [~0.8-1.0] where 1.0 is no filtering and 0.8 is very unique
            separation = 0.85
            
            //featureKind = FeatureKind.Akaze
            
            //reprojection error during 2D feature matching in NDC
            //maxRMSE = 0.025

            //point reprojection error during 3D pose recovery in NDC
            //maxNdcDistance = 0.025

            //similarity threshold for ransac solutions during feature matching
            //ransacThreshold = 0.85
            
            //baseCamera = CameraView.lookAt (V3d(5,0,0)) V3d.Zero V3d.OOI
            
            //reporter = Reporter.Global
            
            //shot must appear in this many triples with other shots
            //triplesPerCamera    = 6
            
            //per-camera number of matches
            //minMatches          = 20
        }

    //path to undistorted photos
    let files =
        System.IO.Directory.GetFiles(@"C:\bla\photo\kermit") 
        |> Array.filter (fun f -> System.IO.Path.GetExtension(f).ToLowerInvariant() = ".png") 
        |> Array.toList

    let sln = PhotogrammetrySolution.createOfFilenames cfg files

    Log.line "Solution:\n"
    for n in sln.nets do    
        Log.line "%d: %d cameras" n.Id n.Cameras.Count
    let ma = (sln.nets |> List.length) - 1

    let slnIdx = AVal.init 0


    //more scene graph stuff: https://github.com/vrvis/Mondo/blob/eec894bd6b7822f9bbbdff28e9ee4c0598d3352a/src/Aardvark.Mondo.SceneGraph/PhotoNetwork.fs#L286
    let pointsSg = 
        let ps = 
            slnIdx |> AVal.map (fun idx -> 
                let i = clamp idx 0 ma
                let n = sln.nets |> List.item i
                Log.line "Solution %d of %d" (i+1) (ma+1)
                n.Points.ValuesAsArray |> Array.map snd |> Array.map V3f
            )

        Sg.draw IndexedGeometryMode.PointList
        |> Sg.vertexAttribute DefaultSemantic.Positions ps
        |> Sg.shader {
            do! DefaultSurfaces.trafo
            do! DefaultSurfaces.constantColor C4f.IndianRed
            do! DefaultSurfaces.pointSprite
            do! DefaultSurfaces.pointSpriteFragment
        }
        |> Sg.uniform "PointSize" (AVal.constant 3.0)

    let camerasSg =
        let cams = 
            slnIdx |> AVal.map (fun idx -> 
                let i = clamp idx 0 ma
                let n = sln.nets |> List.item i
                n.Cameras |> MapExt.toArray |> Array.map snd
            )

        let drawCall =
            cams |> AVal.map (fun a -> 
                DrawCallInfo(
                    FaceVertexCount = 18,
                    InstanceCount = a.Length
                )
            )
           
        let instanceTrafos =
            cams |> AVal.map (fun arr ->
                let trafos = 
                    arr |> Array.map (fun c ->
                        let pp = -c.proj.distortion.principalPoint
                        let trafo = 
                            Trafo3d.ShearXY(pp.X, pp.Y) * 
                            Trafo3d.Scale(1.0, 1.0 / c.proj.aspect, c.proj.focalLength) *
                            c.view.ViewTrafo.Inverse
                        M44f.op_Explicit trafo.Forward
                    )

                ArrayBuffer(trafos) :> IBuffer
            )
                
        Sg.RenderNode(drawCall, IndexedGeometryMode.LineList)
            |> Sg.vertexAttribute' DefaultSemantic.Positions [|
                    V3f.Zero
                    V3f(-1.0f, -1.0f, -1.0f)
                    V3f( 1.0f, -1.0f, -1.0f)
                    V3f( 1.0f,  1.0f, -1.0f)
                    V3f(-1.0f,  1.0f, -1.0f)

                    V3f( 0.0f,  0.0f, -1.0f)
                    V3f( 0.0f,  0.0f,  0.0f)
                |]
            |> Sg.vertexAttribute' DefaultSemantic.Colors [|
                    C4b.Yellow
                    C4b.Yellow
                    C4b.Yellow
                    C4b.Yellow
                    C4b.Yellow
                    
                    C4b.Red
                    C4b.Red
                |]
            |> Sg.index' [|
                    0; 1
                    0; 2
                    0; 3
                    0; 4
                    1; 2
                    2; 3
                    3; 4
                    4; 1

                    5; 6
                |]
            |> Sg.instanceBuffer DefaultSemantic.InstanceTrafo (BufferView(instanceTrafos, typeof<M44f>))
            |> Sg.shader {
                do! DefaultSurfaces.instanceTrafo
                do! DefaultSurfaces.trafo
                do! DefaultSurfaces.thickLine
                do! DefaultSurfaces.thickLineRoundCaps
            }
            |> Sg.uniform "LineWidth" (AVal.constant 2.0)

    use app = new OpenGlApplication()
    use win = app.CreateGameWindow(4)

    let initialView = Aardvark.Rendering.CameraView.lookAt (V3d(6,6,6)) V3d.Zero V3d.OOI
    let view = initialView |> DefaultCameraController.control win.Mouse win.Keyboard win.Time
    let proj = win.Sizes |> AVal.map (fun s -> Frustum.perspective 75.0 0.1 100.0 (float s.X / float s.Y))

    let sg =
        Sg.ofList [pointsSg; camerasSg]
            |> Sg.viewTrafo (view |> AVal.map Aardvark.Rendering.CameraView.viewTrafo)
            |> Sg.projTrafo (proj |> AVal.map Frustum.projTrafo)

    
    let task =
        app.Runtime.CompileRender(win.FramebufferSignature, sg)

    win.Keyboard.Down.Values.Add (fun k -> 
        match k with 
        | Keys.D1 -> transact (fun _ -> slnIdx.Value <- 0)
        | Keys.D2 -> transact (fun _ -> slnIdx.Value <- 1)
        | Keys.D3 -> transact (fun _ -> slnIdx.Value <- 2)
        | Keys.D4 -> transact (fun _ -> slnIdx.Value <- 3)
        | Keys.D5 -> transact (fun _ -> slnIdx.Value <- 4)
        | Keys.D6 -> transact (fun _ -> slnIdx.Value <- 5)
        | Keys.D7 -> transact (fun _ -> slnIdx.Value <- 6)
        | Keys.D8 -> transact (fun _ -> slnIdx.Value <- 7)
        | Keys.D9 -> transact (fun _ -> slnIdx.Value <- 8)
        | Keys.D0 -> transact (fun _ -> slnIdx.Value <- 9)
        | _ -> ()
    )
    win.RenderTask <- task
    win.Run()
    0
