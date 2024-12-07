import yaml
import json


def yaml_to_json(yamlPath,savepath):
    ff=open(savepath,'w')
    with open(yamlPath) as f:
        datas=yaml.load(f, Loader=yaml.FullLoader)
    jsonDatas=json.dumps(datas, indent=6)
    ff.write(jsonDatas)
    ff.close()
    # print(jsonDatas)

yamlfile='39yml.yml'
savep='39j.json'
yaml_to_json(yamlfile,savep)


# data=yaml.safe_load(yamlfile)

# json_data=json.dumps(data, indent=4)

# f=open('7j.json','w')

# f.write(json_data)
# f.close()

