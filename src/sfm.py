import pycolmap, pathlib, time
root=pathlib.Path('/home/claude/sfm'); db=root/'db.db'; img=root/'images'
if db.exists(): db.unlink()
t=time.time()
ro=pycolmap.ImageReaderOptions(); ro.camera_model='SIMPLE_RADIAL'
eo=pycolmap.FeatureExtractionOptions(); eo.sift.max_num_features=4000; eo.num_threads=2
pycolmap.extract_features(db,img,camera_mode=pycolmap.CameraMode.SINGLE,reader_options=ro,extraction_options=eo)
print('extract',time.time()-t,flush=True)
mo=pycolmap.SequentialPairingOptions(); mo.overlap=12; mo.loop_detection=False
pycolmap.match_sequential(db,pairing_options=mo)
print('match',time.time()-t,flush=True)
out=root/'sparse'; out.mkdir(exist_ok=True)
recs=pycolmap.incremental_mapping(db,img,out)
for k,r in recs.items(): print(k,r.summary())
print('done',time.time()-t)
