from openai import OpenAI

def ask(role, prompt):
    api_key = "your key" # 初始化API密钥
    api_base = "https://pro.aiskt.com/v1" # 中转网站网址
    client = OpenAI(api_key=api_key, base_url=api_base)

    completion = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": role}, # 角色扮演词
            {"role": "user", "content": prompt} # 提示词
        ]
    )

    return completion.choices[0].message # 返回响应信息
