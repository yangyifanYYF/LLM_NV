from openai import OpenAI


def ask(role, prompt):
    api_key = "your key"
    api_base = "https://pro.aiskt.com/v1"
    client = OpenAI(api_key=api_key, base_url=api_base)

    completion = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": role},
            {"role": "user", "content": prompt}
        ]
    )

    return completion.choices[0].message
